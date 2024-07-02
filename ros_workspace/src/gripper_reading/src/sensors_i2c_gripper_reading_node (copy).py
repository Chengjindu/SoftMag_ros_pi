#!/usr/bin/env python3
import rospy
from std_msgs.msg import String  # Use String message type
import smbus
import time


# Initialize I2C (SMBus)
bus = smbus.SMBus(1)

# MLX90393 I2C addresses
sensor_addresses = [0x0C, 0x0E]  # 12 in decimal, 0C for 00, 0D for 10, 0E for 01, 0F for 11)
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
        status = bus.read_byte(address)  # Reading back the status byte

        # Configuration for the sensor register 0x02, AH = 0x02, AL = 0xB4, RES for magnetic measurement = 1
        config_register_2 = [0x02, 0xB4, 0x08]
        bus.write_i2c_block_data(address, 0x60, config_register_2)
        status = bus.read_byte(address)  # Reading back the status byte
    except Exception as e:
        rospy.logerr(f"Failed to initialize sensor at address {address}: {e}")
        raise

# Function to start a single measurement
def start_single_measurement(address):
    try:
        # Command to start single measurement with X, Y, Z-Axis enabled
        bus.write_byte(address, 0x3E)
        status = bus.read_byte(address)  # Reading back the status byte
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


# Main function to manage the connection and send data
def main():
    try:
        rate = rospy.Rate(10)  # 10 Hz, adjust as needed
        while not rospy.is_shutdown():
            for addr in sensor_addresses:
                start_single_measurement(addr)
                xMag, yMag, zMag = read_magnetic_field(addr)
                sensor_label = sensor_labels[addr]  # Unique identifier for each sensor

                # Format the message as a string
                message_str = f"{sensor_label},{xMag},{yMag},{zMag}"
                message = String(data=message_str)
                # rospy.loginfo("Received %s", message)  # Log received data for debugging
                sensor_publishers.publish(message)

                rate.sleep()  # Sleep to maintain the desired loop rate
    except KeyboardInterrupt:
        print("Data transmission stopped by user.")
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node('sensor_i2c_reading_node', anonymous=True)

    # Create a ROS publisher for each sensor
    sensor_publishers = rospy.Publisher('sensor_data', String, queue_size=10)


    for addr in sensor_addresses:
        init_mlx90393(addr)

    main()