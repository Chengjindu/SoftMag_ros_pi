#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import sys
import rospy
from std_msgs.msg import String, Int32, Bool
import json
import signal

GPIO.setwarnings(False)

# Motor GPIO Pins
out1 = 17
out2 = 18
out3 = 27
out4 = 22
gpio_setup_done = False
position_counter = 0
steps_per_rev = 200
full_stroke_angle = 85
max_steps = int((full_stroke_angle / 360) * steps_per_rev)
step_sleep = 0.01
motor_rpm = 2
initialize_flag = True
motor_running = False  # Motor operating flag

# Initializing publishers
global motor_pos_reading_publisher, motor_stop_publisher, motor_zero_publisher
motor_pos_reading_publisher = 0
motor_stop_publisher = None
motor_zero_publisher = None


def setup_gpio():  # Setup GPIO pins for motor control
    global gpio_setup_done

    if not gpio_setup_done:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(out1, GPIO.OUT)
        GPIO.setup(out2, GPIO.OUT)
        GPIO.setup(out3, GPIO.OUT)
        GPIO.setup(out4, GPIO.OUT)
        GPIO.output(out1, False)
        GPIO.output(out2, False)
        GPIO.output(out3, False)
        GPIO.output(out4, False)
        gpio_setup_done = True


def step_motor(steps, step_sleep, direction):  # Function for actuating the motor
    global position_counter, initialize_flag, motor_running, motor_stop_publisher, motor_zero_publisher

    initialize_flag = False  # Prevent initializing the motor again
    motor_running = True

    motor_stop_publisher.publish(Bool(data=False))

    for i in range(steps):

        if not motor_running:
            rospy.loginfo("Motor stop requested.")
            break  # Exit if motor_running is cleared

        if (direction == 'forward' and position_counter >= max_steps) or \
                (direction == 'backward' and position_counter <= 0):
            rospy.loginfo("Safety limit reached.")
            motor_running = False
            motor_stop_publisher.publish(Bool(data=True))
            return

        if direction == 'forward':
            sequence = i % 4
            position_counter += 1
        else:
            sequence = (4 - (i % 4)) % 4
            position_counter -= 1

        if position_counter == 0:
            motor_zero_publisher.publish(Bool(data=True))
        else:
            motor_zero_publisher.publish(Bool(data=False))

        if sequence == 0:
            GPIO.output(out4, GPIO.HIGH)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.LOW)
        elif sequence == 1:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.HIGH)
            GPIO.output(out1, GPIO.LOW)
        elif sequence == 2:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.HIGH)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.LOW)
        elif sequence == 3:
            GPIO.output(out4, GPIO.LOW)
            GPIO.output(out3, GPIO.LOW)
            GPIO.output(out2, GPIO.LOW)
            GPIO.output(out1, GPIO.HIGH)

        time.sleep(step_sleep)

        motor_pos_reading_publisher.publish(position_counter)

    motor_running = False
    motor_stop_publisher.publish(Bool(data=True))


def motor_speed_ctrl_callback(data):  # Callback for motor speed control
    global motor_rpm, step_sleep

    motor_rpm = data.data
    step_sleep = 60.0 / (motor_rpm * steps_per_rev)  # Update motor speed


def sensors_stabilized_callback(data):  # Callback for sensors stabilized
    global motor_rpm, step_sleep, initialize_flag

    sensors_stabilized = data.data

    if sensors_stabilized and initialize_flag:
        initialize_flag = False


def motor_pos_ctrl_callback(data):  # Callback for motor control input
    global step_sleep, initialize_flag

    position = data.data
    current_position = position_counter

    if not initialize_flag:
        if position == current_position:
            rospy.loginfo("Motor is already at the desired position.")
            motor_stop_publisher.publish(json.dumps({'motor_stop': True}))
            return

        if position > current_position:
            steps = position - current_position
            direction = 'forward'
        else:
            steps = current_position - position
            direction = 'backward'

        step_motor(steps, step_sleep, direction)


def stop_all_callback(data):  # Callback for emergency stop
    global motor_running

    if data.data:
        motor_running = False


def zero_motor_callback(data):  # Callback for zeroing motor
    global motor_running

    if data.data:
        motor_running = False
        return_to_zero()
    setup_gpio()


def return_to_zero():  # Function for returning the motor to zero
    global position_counter, step_sleep, gpio_setup_done, motor_running, motor_zero_publisher

    motor_running = False

    try:
        if position_counter != 0:
            rospy.loginfo("Returning to Zero Position...")
            setup_gpio()  # Ensure GPIO setup before moving motor
            # Determine direction to return to zero
            direction = 'backward' if position_counter > 0 else 'forward'
            step_motor(abs(position_counter), step_sleep, direction)

    finally:
        rospy.loginfo("Cleaning up GPIO...")
        if gpio_setup_done:
            GPIO.cleanup()
            motor_zero_publisher.publish(Bool(data=True))
            gpio_setup_done = False  # Reset flag after cleanup


def signal_handler(sig, frame):  # Function for handling gracefully shut down
    rospy.loginfo('SIGINT caught, initiating cleanup...')
    return_to_zero()
    sys.exit(0)


def motor_control_node():  # Main function to initialize the motor control node
    global motor_pos_reading_publisher, motor_stop_publisher, motor_zero_publisher

    rospy.init_node('testing_motor_control_node', anonymous=True)
    setup_gpio()

    motor_pos_reading_publisher = rospy.Publisher('motor_pos_reading', Int32, queue_size=10, latch=True)
    motor_stop_publisher = rospy.Publisher('motor_stop', Bool, queue_size=10, latch=True)
    motor_zero_publisher = rospy.Publisher('motor_zero', Bool, queue_size=10, latch=True)

    signal.signal(signal.SIGINT, signal_handler)

    rospy.Subscriber('motor_speed_ctrl', Int32, motor_speed_ctrl_callback)
    rospy.Subscriber('motor_pos_ctrl', Int32, motor_pos_ctrl_callback)
    rospy.Subscriber('sensors_stabilized', Bool, sensors_stabilized_callback)
    rospy.Subscriber('zero_motor', Bool, zero_motor_callback)
    rospy.Subscriber('stop_all', Bool, stop_all_callback)

    motor_zero_publisher.publish(Bool(data=True))
    motor_stop_publisher.publish(Bool(data=True))

    rospy.spin()


if __name__ == "__main__":
    motor_control_node()
