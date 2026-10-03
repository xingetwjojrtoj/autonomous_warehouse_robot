#!/bin/bash
# Source ROS 2 system installation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
# Source custom ROS 2 workspace if applicable (uncomment and update path):
# source /home/gyrotronix/ros2_ws/install/setup.bash

# Execute Python server
exec /usr/bin/python3 /home/gyrotronix/GUI/server.py
