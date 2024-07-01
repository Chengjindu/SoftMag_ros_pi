#!/bin/bash

source /home/chengjindu/SoftMag/Console/ros_workspace/start_env_loader_pi.sh

# Ensure ROS master is running
until rostopic list; do
  echo "Waiting for ROS master to start..."
  sleep 2
done

# Get the value of the default_pi_mode parameter
DEFAULT_PI_MODE=$(rosparam get /default_pi_mode)

if [ "$DEFAULT_PI_MODE" == "sensor" ]; then
  roslaunch launch_console_pi sensor_launch.launch
elif [ "$DEFAULT_PI_MODE" == "gripper_automatic" ] || [ "$DEFAULT_PI_MODE" == "gripper_testing" ]; then
  roslaunch launch_console_pi gripper_launch.launch
else
  echo "Unknown mode: $DEFAULT_PI_MODE. Please set the /default_pi_mode parameter to either 'sensor' or 'gripper_automatic' or 'gripper_testing'."
  exit 1
fi

