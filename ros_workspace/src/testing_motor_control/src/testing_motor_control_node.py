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
initialize_flag = True
motor_running = False

global motor_pos_reading_publisher
motor_pos_reading_publisher = 0

global motor_stop_publisher
motor_stop_publisher = None

global motor_zero_publisher
motor_zero_publisher = None

def setup_gpio():
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

def step_motor(steps, step_sleep, direction):
    global position_counter, initialize_flag, motor_running, motor_stop_publisher, motor_zero_publisher
    
    initialize_flag = False  # Prevent initializing the motor again
    motor_running = True  # Indicate motor is starting
    
    motor_stop_publisher.publish(Bool(data=False))  # Indicate motor is running
    
    for i in range(steps):
        
        if not motor_running:
            rospy.loginfo("Motor stop requested.")
            break  # Exit if motor_running is cleared
        
        if (direction == 'forward' and position_counter >= max_steps) or \
           (direction == 'backward' and position_counter <= 0):
            rospy.loginfo("Safety limit reached. Stopping motor.")
            motor_running = False  # Indicate motor has stopped
            motor_stop_publisher.publish(Bool(data=True))  # Indicate motor has stopped
            return
        
        if direction == 'forward':
            sequence = i % 4
            position_counter += 1  # Increment position counter
        else:  # Reverse direction
            sequence = (4 - (i % 4)) % 4
            position_counter -= 1  # Decrement position counter
            
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
        
    motor_running = False  # Indicate motor has stopped
    motor_stop_publisher.publish(Bool(data=True))

def return_to_zero():
    rospy.loginfo("Cleaning up GPIO and stopping motor...")
    global position_counter, step_sleep, gpio_setup_done, motor_running, motor_zero_publisher
    motor_running = False
    try:
        if position_counter != 0:
            rospy.loginfo("Returning to Zero Position...")
            rospy.loginfo("Current position = {}".format(position_counter))
            # Ensure GPIO setup before moving motor
            setup_gpio()
            # Determine direction to return to zero
            direction = 'backward' if position_counter > 0 else 'forward'
            step_motor(abs(position_counter), step_sleep, direction)
    finally:
        if gpio_setup_done:
            GPIO.cleanup()
            motor_zero_publisher.publish(Bool(data=True))  # Initialize motor zero status
            gpio_setup_done = False  # Reset flag after cleanup

def sensors_stabilized_callback(data):
    global step_sleep, initialize_flag
    
    sensors_stabilized = data.data
    
    if sensors_stabilized and initialize_flag:
        # Assuming the RPM is predefined for this demonstration
        rpm = 2  # Modify as needed
        step_sleep = 60 / (rpm * steps_per_rev)
        # Move forward to close the fingers
        rospy.loginfo(f"Sensor initialized, start closing with {rpm} rpm speed...")
        initialize_flag = False

def motor_pos_ctrl_callback(data):
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
    else:
        rospy.loginfo("Waiting for sensors to stablize before moving motor.")

def stop_all_callback(data):
    global motor_running
    
    if data.data:
        motor_running = False  # Request motor stop
        rospy.loginfo(f"Motor stopped at position {position_counter}")

def zero_motor_callback(data):
    global motor_running
    
    if data.data:
        motor_running = False  # Request motor stop
        return_to_zero()
    setup_gpio()

def signal_handler(sig, frame):
    rospy.loginfo('SIGINT caught, initiating cleanup...')
    return_to_zero()
    sys.exit(0)

def motor_control_node():
    global motor_pos_reading_publisher, motor_stop_publisher, motor_zero_publisher
    
    rospy.init_node('testing_motor_control_node', anonymous=True)
    setup_gpio()
    
    motor_pos_reading_publisher = rospy.Publisher('motor_pos_reading', Int32, queue_size=10, latch=True)
    motor_stop_publisher = rospy.Publisher('motor_stop', Bool, queue_size=10, latch=True)
    motor_zero_publisher = rospy.Publisher('motor_zero', Bool, queue_size=10, latch=True)
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    rospy.Subscriber('motor_pos_ctrl', Int32, motor_pos_ctrl_callback)
    rospy.Subscriber('sensors_stabilized', Bool, sensors_stabilized_callback)
    rospy.Subscriber('zero_motor', Bool, zero_motor_callback)
    rospy.Subscriber('stop_all', Bool, stop_all_callback)
    
    motor_zero_publisher.publish(Bool(data=True))  # Initialize motor zero status
    motor_stop_publisher.publish(Bool(data=True))  # Initialize motor stop status

    rospy.spin()
    
if __name__ == "__main__":
    motor_control_node()
