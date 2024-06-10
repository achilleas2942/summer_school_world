#!/bin/bash
# Source the ROS setup script
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Execute the provided command
exec "$@"
