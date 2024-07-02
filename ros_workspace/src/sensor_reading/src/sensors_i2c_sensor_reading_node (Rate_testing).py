#!/usr/bin/env python3
import rospy
from std_msgs.msg import String  # Use String message type
import smbus
import threading
import time


# Initialize I2C (SMBus)
bus = smbus.SMBus(1)

# MLX90393 I2C addresses
sensor_addresses = [0x0D, 0x0F]  # 12 in decimal, 0C for 00, 0D for 10, 0E for 01, 0F for 11)
# Create a dictionary to map sensor addresses to labels
sensor_labels = {addr: f"S{i+1}" for i, addr in enumerate(sensor_addresses)}

# Server details
SERVER_IP = "10.255.32.70"  # Replace 'desktop_IP' with the actual IP address of your desktop
SERVER_PORT = 65432         # The port should match the server's listening port

# Function to initialize the MLX90393 sensor
def init_mlx90393(address):
    try:
        # Configuration for the sensor register 0x00, AH = 0x00, AL = 0x7C, GAIN_SEL = 7 (1x gain)
        config_register_0 = [0x00, 0x7C, 0x00]
        bus.write_i2c_block_data(address, 0x60, config_register_0)
        bus.read_byte(address)  # Reading back the status byte

        # Configuration for the sensor register 0x02, AH = 0x02, AL = 0xB4, RES for magnetic measurement = 1
        config_register_2 = [0x02, 0xB4, 0x08]
        bus.write_i2c_block_data(address, 0x60, config_register_2)
        bus.read_byte(address)  # Reading back the status byte

        rospy.loginfo(f"Sensor at address {address} initialized successfully")

    except Exception as e:
        rospy.logerr(f"Failed to initialize sensor at address {address}: {e}")
        raise

# Function to start a single measurement
def start_single_measurement(address):
    try:
        # Command to start single measurement with X, Y, Z-Axis enabled
        bus.write_byte(address, 0x3E)
        bus.read_byte(address)  # Reading back the status byte
        time.sleep(0.01)  # Wait for the measurement to complete

    except Exception as e:
        rospy.logerr(f"Failed to start measurement for sensor at address {address}: {e}")
        raise

# Function to read, convert and filter the magnetic field data
def read_magnetic_field(address):
    try:
        # Read 7 bytes of data from address 0x4E
        data = bus.read_i2c_block_data(address, 0x4E, 7)

        xMag = float(data[1] * 256 + data[2])
        if xMag > 32767:
            xMag -= 65536
        xMag *= 0.524 * 0.01  # μT - Gauss

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
        rospy.logerr(f"Failed to read magnetic field data from sensor at address {address}: {e}")
        return (0.0, 0.0, 0.0)  # Return default values in case of failure

def read_and_publish_data(address, publisher, rate):
    rate_limiter = rospy.Rate(rate)
    count = 0
    start_time = time.time()

    while not rospy.is_shutdown():
        try:
            start_single_measurement(address)
            xMag, yMag, zMag = read_magnetic_field(address)
            sensor_label = sensor_labels[address]  # Unique identifier for each sensor

            message_str = f"{sensor_label},{xMag},{yMag},{zMag}"
            message = String(data = message_str)
            publisher.publish(message)

            count += 1
            rate_limiter.sleep()

        except Exception as e:
            rospy.logerr(f"Error in sensor reading loop for address {address}: {e}")
            break

    elapsed_time = time.time() - start_time
    if elapsed_time > 0:
        rospy.loginfo(f"Measured rate for sensor at address {address}: {count / elapsed_time} Hz")
    else:
        rospy.logwarn(f"Elapsed time is zero for sensor at address {address}")
    return count / elapsed_time if elapsed_time > 0 else 0

if __name__ == "__main__":

    rospy.init_node('sensor_i2c_reading_node', anonymous=True)   # Initialize ROS node

    # Create a ROS publisher for each sensor
    sensors_publishers = {addr: rospy.Publisher(f'sensor_data_{sensor_labels[addr]}', String, queue_size=10) for addr in sensor_addresses}

    for addr in sensor_addresses:   # Initialize sensors
        init_mlx90393(addr)

    duration = 10  # Measurement duration in seconds
    measured_rates = []

    threads = []    # Start a thread for each sensor to read and publish data concurrently

    # Start a thread for each sensor to read and publish data concurrently
    def measure_and_store_rate(addr):
        rate = read_and_publish_data(addr, sensors_publishers[addr], 1000)
        measured_rates.append((addr, rate))

    for addr in sensor_addresses:
        thread = threading.Thread(target=measure_and_store_rate, args=(addr,))
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    for addr, rate in measured_rates:
        rospy.loginfo(f"Maximum data reading rate for sensor at address {addr}: {rate} Hz")

    rospy.spin()  # Keep the main thread alive

