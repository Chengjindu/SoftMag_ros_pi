#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import smbus
import threading
import time
from concurrent.futures import ThreadPoolExecutor

# Initialize I2C (SMBus)
bus = smbus.SMBus(1)

# MLX90393 I2C addresses
sensor_addresses = [0x0D, 0x0F]
sensor_labels = {addr: f"S{i+1}" for i, addr in enumerate(sensor_addresses)}

publish_rate = 100  # Sensor reading rate for the system
retries = 5
delay = 0.008       # Shouldn't go below 0.0075

def init_mlx90393(address):     # Function for initializing the MLX90393 sensor at the given I2C address
    for attempt in range(retries):
        try:
            # Configuration for the sensor register 0x00, AH = 0x00, AL = 0x7C, GAIN_SEL = 7 (1x gain)
            config_register_0 = [0x00, 0x7C, 0x00]
            bus.write_i2c_block_data(address, 0x60, config_register_0)
            status = bus.read_byte(address)  # Reading back the status byte

            # Configuration for the sensor register 0x02, AH = 0x02, AL = 0xB4, RES for magnetic measurement = 1
            config_register_2 = [0x02, 0xB4, 0x08]
            bus.write_i2c_block_data(address, 0x60, config_register_2)
            status = bus.read_byte(address)  # Reading back the status byte
            return

        except Exception as e:
            rospy.logwarn(f"Retry {attempt + 1}/{retries} failed for initializing sensor at address {address}: {e}")
            if attempt == retries - 1:
                rospy.logerr(f"Failed to initialize sensor at address {address} after {retries} retries: {e}")
                raise

def start_single_measurement(address):      # Function to start a single measurement on the MLX90393
    for attempt in range(retries):
        try:
            # Command to start single measurement with X, Y, Z-Axis enabled
            bus.write_byte(address, 0x3E)
            bus.read_byte(address)  # Reading back the status byte
            time.sleep(delay)  # Wait for the measurement to complete, important paramerter to define
            return
        except Exception as e:
            rospy.logwarn(f"Retry {attempt + 1}/{retries} failed for starting measurement at address {address}: {e}")
            if attempt == retries - 1:
                rospy.logerr(f"Failed to start measurement for sensor at address {address} after {retries} retries: {e}")
                raise

def read_magnetic_field(address):           # Function for reading magnetic flux data from the MLX90393 sensor
    for attempt in range(retries):
        try:
            data = bus.read_i2c_block_data(address, 0x4E, 7)

            xMag = float(data[1] * 256 + data[2])
            if xMag > 32767:
                xMag -= 65536
            xMag *= 0.300 * 0.01

            yMag = float(data[3] * 256 + data[4])
            if yMag > 32767:
                yMag -= 65536
            yMag *= 0.300 * 0.01

            zMag = float(data[5] * 256 + data[6])
            if zMag > 32767:
                zMag -= 65536
            zMag *= 0.484 * 0.01

            return (xMag, yMag, zMag)
        except Exception as e:
            rospy.logwarn(f"Retry {attempt + 1}/{retries} failed for reading magnetic field data from sensor at address {address}: {e}")
            if attempt == retries - 1:
                rospy.logerr(f"Failed to read magnetic field data from sensor at address {address} after {retries} retries: {e}")
                return (0.0, 0.0, 0.0)

def read_and_publish_data(address, publisher, rate, stop_event):    # Function for reading and publishing data to the ROS network
    rate_limiter = rospy.Rate(rate)

    while not rospy.is_shutdown() and not stop_event.is_set():
        try:
            start_single_measurement(address)
            xMag, yMag, zMag = read_magnetic_field(address)

            message = Float32MultiArray()
            message.data = [xMag, yMag, zMag]
            publisher.publish(message)

            # Print the magnetic field values to the console
            # print(f"X={xMag:.2f}uT, Y={yMag:.2f}uT, Z={zMag:.2f}uT")

            rate_limiter.sleep()

        except Exception as e:
            rospy.logerr(f"Error in sensor reading loop for address {address}: {e}")
            break

if __name__ == "__main__":      # Main process

    rospy.init_node('sensor_reading_node', anonymous=True)  # Initialize ROS node

    # Create a ROS publisher for each sensor
    sensors_publishers = {addr: rospy.Publisher(f'sensor_data_{sensor_labels[addr]}', Float32MultiArray, queue_size=20) for addr in sensor_addresses}

    for addr in sensor_addresses:  # Initialize sensors
        init_mlx90393(addr)

    stop_event = threading.Event()

    # Use ThreadPoolExecutor to read and publish data from sensors concurrently
    with ThreadPoolExecutor(max_workers=len(sensor_addresses)) as executor:
        for addr in sensor_addresses:
            executor.submit(read_and_publish_data, addr, sensors_publishers[addr], publish_rate, stop_event) # Best 100-110Hz

        rospy.spin()  # Keep the main thread alive

    stop_event.set()
    rospy.loginfo("Measurement complete. Stopping sensor reading threads.")