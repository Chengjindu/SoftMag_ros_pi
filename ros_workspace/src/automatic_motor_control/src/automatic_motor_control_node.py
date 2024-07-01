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
start_sleep = 2
start_flag = False
release_flag = True
motor_running = False

global motor_stop_publisher
motor_stop_publisher = None

global motor_pos_reading_publisher
motor_pos_reading_publisher = 0

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
    global position_counter, start_flag, motor_running, motor_stop_publisher, motor_zero_publisher
    
    start_flag = False	  # Prevent starting the motor again
    motor_running = True  # Indicate motor is starting
    
    for i in range(steps):
        
        if not motor_running:
            rospy.loginfo("Motor stop requested.")
            break  # Exit if motor_running is cleared
        
        if (direction == 'forward' and position_counter >= max_steps) or \
           (direction == 'backward' and position_counter <= 0):
            rospy.loginfo("Safety limit reached. Stopping motor.")
            motor_running = False  # Indicate motor has stopped
            motor_stop_publisher.publish(Bool(data=True))
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

def sensors_stabilized_callback(data):
    global step_sleep, start_flag
    
    sensors_stabilized = data.data
    
    if sensors_stabilized and start_flag:
        rpm = 2  # Modify as needed
        step_sleep = 60 / (rpm * steps_per_rev)
        rospy.loginfo(f"Sensor initialized, start closing with {rpm} rpm speed...")
        time.sleep(start_sleep)
        step_motor(max_steps, step_sleep, 'forward')

def contact_trigger_callback(data):
    if data.data:
        rospy.loginfo("Contact detected. Stopping motor.")
        stop_motor_after_contact()  # Stops the motor

def stop_motor_after_contact():
    global motor_running
    motor_running = False  # Request motor stop
    rospy.loginfo(f"Motor stopped at position {position_counter}")

def release_finish_callback(data):
    global release_flag
    if data.data and release_flag:
        rospy.loginfo('Release finished, returning to zero...')
        return_to_zero()
        release_flag = False

def return_to_zero():
    global position_counter, step_sleep, gpio_setup_done, motor_running, motor_zero_publisher
    
    try:
        if position_counter != 0:
            rospy.loginfo("Current position = {}, Returning to Zero Position...".format(position_counter))   
            setup_gpio() # Ensure GPIO setup before moving motor   
            direction = 'backward' if position_counter > 0 else 'forward' # Determine direction to return to zero
            step_motor(abs(position_counter), step_sleep, direction)
            
    finally:
        if gpio_setup_done:
            rospy.loginfo("Cleaning up GPIO...")
            GPIO.cleanup()
            motor_zero_publisher.publish(Bool(data=True))
            gpio_setup_done = False  # Reset flag after cleanup

def restart_callback(data):
    global start_flag, release_flag
    start_flag = True
    release_flag = True
    setup_gpio()
    rospy.loginfo("Restartting...")

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
    
    rospy.init_node('automatic_motor_control_node', anonymous=True)
    setup_gpio()
    
    motor_pos_reading_publisher = rospy.Publisher('motor_pos_reading', Int32, queue_size=10, latch=True)
    motor_stop_publisher = rospy.Publisher('motor_stop', Bool, queue_size=10, latch=True)
    motor_zero_publisher = rospy.Publisher('motor_zero', Bool, queue_size=10, latch=True)
    
    signal.signal(signal.SIGINT, signal_handler)    # Register signal handler
    
    rospy.Subscriber('contact_trigger', Bool, contact_trigger_callback)
    rospy.Subscriber('release_finish', Bool, release_finish_callback)
    rospy.Subscriber('restart', Bool, restart_callback)
    rospy.Subscriber('stop_all', Bool, stop_all_callback)
    rospy.Subscriber('zero_motor', Bool, zero_motor_callback)
    rospy.Subscriber('sensors_stabilized', Bool, sensors_stabilized_callback)
    
    motor_zero_publisher.publish(Bool(data=True))
    motor_stop_publisher.publish(Bool(data=True))
    
    rospy.spin()

if __name__ == "__main__":
    motor_control_node()
