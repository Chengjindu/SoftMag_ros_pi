#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import sys
import rospy
from std_msgs.msg import String, Int32
import json
import signal

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
start_flag = True
motor_running = False
automaticMode = True

global motor_stop_publisher
motor_stop_publisher = None

global motor_pos_reading_publisher
motor_pos_reading_publisher = 0


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
    global position_counter, start_flag, motor_running
    
    start_flag = False	  # Prevent starting the motor again
    motor_running = True  # Indicate motor is starting
    
    for i in range(steps):
        
        if not motor_running:
            rospy.loginfo("Motor stop requested.")
            break  # Exit if motor_running is cleared
        
        if (direction == 'forward' and position_counter >= max_steps) or \
           (direction == 'backward' and position_counter <= 0):
            rospy.loginfo("Safety limit reached. Stopping motor.")
            return
        
        if direction == 'forward':
            sequence = i % 4
            position_counter += 1  # Increment position counter
        else:  # Reverse direction
            sequence = (4 - (i % 4)) % 4
            position_counter -= 1  # Decrement position counter
            
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

def return_to_zero():
    rospy.loginfo("Cleaning up GPIO and stopping motor...")
    global position_counter, step_sleep, gpio_setup_done, motor_running
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
            gpio_setup_done = False  # Reset flag after cleanup

def stop_motor_after_contact():
    global motor_running, motor_stop_publisher
    motor_running = False  # Request motor stop
    rospy.loginfo(f"Motor stopped at position {position_counter}")
    motor_stop_publisher.publish(json.dumps({'motor_stop': True}))

def processed_sensor_data_callback(data):
    global step_sleep, start_flag
    
    if not automaticMode:
        return  # Ignore in testing mode
    
    data_json = json.loads(data.data)
    
    if data_json.get('stable_flag') and start_flag:
        # Assuming the RPM is predefined for this demonstration
        rpm = 2  # Modify as needed
        step_sleep = 60 / (rpm * steps_per_rev)
        # Move forward to close the fingers
        rospy.loginfo(f"Sensor initialized, start closing with {rpm} rpm speed...")
        step_motor(max_steps, step_sleep, 'forward')

def contact_detected_callback(data):
    
    if not automaticMode:
        return  # Ignore in testing mode
    
    data_json = json.loads(data.data)
    
    if data_json.get('change_flag'):
        rospy.loginfo("Contact detected. Stopping motor.")
        stop_motor_after_contact()  # Stops the motor

def grasp_stable_callback(data):
    
    if not automaticMode:
        return  # Ignore in testing mode
    
    data_json = json.loads(data.data)
    
    if data_json.get('grasp_stable_flag'):
        motor_stop_publisher.publish(json.dumps({'motor_stop': False}))

def release_finish_callback(data):
    data_json = json.loads(data.data)
    
    if data_json.get('release_finish_flag'):
        rospy.loginfo('Release finished, returning to zero...')
        return_to_zero()
    motor_stop_publisher.publish(json.dumps({'motor_stop': False}))

def motor_pos_ctrl_callback(data):
    global step_sleep
    position = data.data
    current_position = position_counter
    
    if position > current_position:
        steps = position - current_position
        direction = 'forward'
    else:
        steps = current_position - position
        direction = 'backward'
    
    step_motor(steps, step_sleep, direction)

def ctrl_mode_callback(data):
    global automaticMode
    data_json = json.loads(data.data)
    ctrl_mode = data_json.get('ctrl_mode')
    if ctrl_mode == "automatic":
        automaticMode = True
        rospy.loginfo("Switched to automatic mode.")
    elif ctrl_mode == "testing":
        automaticMode = False
        rospy.loginfo("Switched to testing mode.")

def signal_handler(sig, frame):
    rospy.loginfo('SIGINT caught, initiating cleanup...')
    return_to_zero()  # Assuming this function performs the motor reset and GPIO cleanup
    sys.exit(0)

def motor_control_node():
    global motor_pos_reading_publisher, motor_stop_publisher  
    
    rospy.init_node('motor_control_node', anonymous=True)
    setup_gpio()
    
    # Define a publisher for the "motor_stop" topic
    motor_stop_publisher = rospy.Publisher('motor_stop', String, queue_size=10, latch=True)
    
    # Define a publisher for the "motor_pos_reading" topic
    motor_pos_reading_publisher = rospy.Publisher('motor_pos_reading', Int32, queue_size=10, latch=True)
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    rospy.Subscriber('ctrl_mode', String, ctrl_mode_callback)
    rospy.Subscriber('motor_pos_ctrl', Int32, motor_pos_ctrl_callback)
    rospy.Subscriber('processed_sensor_data', String, processed_sensor_data_callback)
    rospy.Subscriber('contact_detect', String, contact_detected_callback)
    rospy.Subscriber('grasp_stable', String, grasp_stable_callback)
    rospy.Subscriber('release_finish', String, release_finish_callback)
    rospy.Subscriber('restart', String, restart_callback)
    rospy.Subscriber('stop_all', String, stop_all_callback)

    rospy.spin()
    
def restart_callback(msg):
    global start_flag
    start_flag = True
    setup_gpio()
    rospy.loginfo("Restartting...")

def stop_all_callback(data):
    global motor_running, motor_stop_publisher
    
    data_json = json.loads(data.data)
    if data_json.get('stop_all_flag'):
        motor_running = False  # Request motor stop
        rospy.loginfo(f"Motor stopped at position {position_counter}")
    
    motor_stop_publisher.publish(json.dumps({'motor_stop': False}))



if __name__ == "__main__":
    motor_control_node()

