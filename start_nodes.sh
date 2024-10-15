#!/bin/bash

# Source ROS 2 setup files
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash

# Launch multiple nodes
ros2 run my_wheelchair mapping_node &
ros2 run my_wheelchair navigation_node &
ros2 run my_wheelchair voice_recognition_node &
ros2 run my_wheelchair navigation_feedback_node &
ros2 run my_wheelchair voice_feedback_node &

# Wait for all background processes to finish
wait