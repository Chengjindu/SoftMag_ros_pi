#!/usr/bin/env python3
import rospy
from std_msgs.msg import String  # Use String message type
import smbus
import threading
import time
from concurrent.futures import ThreadPoolExecutor

# Initialize I2C (SMBus)
bus = smbus.SMBus(1)

# MLX90393 I2C addresses
sensor_addresses = [0x0C, 0x0E]
sensor_labels = {addr: f"S{i+1}" for i, addr in enumerate(sensor_addresses)}

publish_rate = 80
retries = 5

def init_mlx90393(address):
    for attempt in range(retries):
        try:
            config_register_0 = [0x00, 0x7C, 0x00]
            bus.write_i2c_block_data(address, 0x60, config_register_0)
            time.sleep(0.01)
            bus.read_byte(address)
            time.sleep(0.01)

            config_register_2 = [0x02, 0xB4, 0x08]
            bus.write_i2c_block_data(address, 0x60, config_register_2)
            time.sleep(0.01)
            bus.read_byte(address)
            rospy.loginfo(f"Sensor at address {address} initialized successfully")
            return

        except Exception as e:
            rospy.logwarn(f"Retry {attempt + 1}/{retries} failed for initializing sensor at address {address}: {e}")
            if attempt == retries - 1:
                rospy.logerr(f"Failed to initialize sensor at address {address} after {retries} retries: {e}")
                raise

def start_single_measurement(address):
    for attempt in range(retries):
        try:
            bus.write_byte(address, 0x3E)
            # time.sleep(0.01)
            bus.read_byte(address)
            return  # Exit if measurement start is successful
        except Exception as e:
            rospy.logwarn(f"Retry {attempt + 1}/{retries} failed for starting measurement at address {address}: {e}")
            if attempt == retries - 1:
                rospy.logerr(f"Failed to start measurement for sensor at address {address} after {retries} retries: {e}")
                raise

def read_magnetic_field(address):
    for attempt in range(retries):
        try:
            data = bus.read_i2c_block_data(address, 0x4E, 7)

            xMag = float(data[1] * 256 + data[2])
            if xMag > 32767:
                xMag -= 65536
            xMag *= 0.524 * 0.01

            yMag = float(data[3] * 256 + data[4])
            if yMag > 32767:
                yMag -= 65536
            yMag *= 0.524 * 0.01

            zMag = float(data[5] * 256 + data[6])
            if zMag > 32767:
                zMag -= 65536
            zMag *= 0.845 * 0.01

            return (xMag, yMag, zMag)
        except Exception as e:
            rospy.logwarn(f"Retry {attempt + 1}/{retries} failed for reading magnetic field data from sensor at address {address}: {e}")
            if attempt == retries - 1:
                rospy.logerr(f"Failed to read magnetic field data from sensor at address {address} after {retries} retries: {e}")
                return (0.0, 0.0, 0.0)

def read_and_publish_data(address, publisher, rate, stop_event):
    rate_limiter = rospy.Rate(rate)

    while not rospy.is_shutdown() and not stop_event.is_set():
        try:
            start_single_measurement(address)
            xMag, yMag, zMag = read_magnetic_field(address)
            sensor_label = sensor_labels[address]

            message_str = f"{sensor_label},{xMag},{yMag},{zMag}"
            message = String(data=message_str)
            publisher.publish(message)

            # rate_limiter.sleep()

        except Exception as e:
            rospy.logerr(f"Error in sensor reading loop for address {address}: {e}")
            break

if __name__ == "__main__":

    rospy.init_node('sensor_i2c_reading_node', anonymous=True)  # Initialize ROS node

    # Create a ROS publisher for each sensor
    sensors_publishers = {addr: rospy.Publisher(f'sensor_data_{sensor_labels[addr]}', String, queue_size = 20) for addr in sensor_addresses}

    for addr in sensor_addresses:  # Initialize sensors
        init_mlx90393(addr)

    stop_event = threading.Event()

    with ThreadPoolExecutor(max_workers=len(sensor_addresses)) as executor:
        for addr in sensor_addresses:
            executor.submit(read_and_publish_data, addr, sensors_publishers[addr], publish_rate, stop_event) # Best 79-85Hz

        rospy.spin()  # Keep the main thread alive

    stop_event.set()
    rospy.loginfo("Measurement complete. Stopping sensor reading threads.")