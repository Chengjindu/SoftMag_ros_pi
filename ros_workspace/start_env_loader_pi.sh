#!/bin/bash

# Define the absolute path to the configuration file
CONFIG_FILE="/home/chengjindu/SoftMag/Console/ros_workspace/start_config_pi.json"

# Ensure the CONFIG_FILE exists
if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "Configuration file $CONFIG_FILE not found!"
  exit 1
fi

# Load configuration parameters
ROS_MASTER_URI=$(jq -r '.ros_master_uri' "$CONFIG_FILE")
export ROS_MASTER_URI
echo "ROS_MASTER_URI: $ROS_MASTER_URI"

ROS_IP=$(jq -r '.pi_ip' "$CONFIG_FILE")
export ROS_IP
echo "ROS_IP: $ROS_IP"

# Set permissions for /dev/gpiomem
sudo chown root.gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem

# Source ROS and workspace setup scripts
source ~/.bashrc
source /home/chengjindu/ros_catkin_ws/install_isolated/setup.bash
source /home/chengjindu/SoftMag/Console/ros_workspace/devel/setup.bash
