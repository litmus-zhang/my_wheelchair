#!/bin/bash
sudo apt-get update
sudo apt update -y
sudo apt install python3-pip portaudio19-dev
pip3 install SpeechRecognition pyaudio pyttsx3 RPi.GPIO

# Source ROS 2 setup files
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash

# make the fedback scripts executable
chmod +x my_wheelchair/scripts/voice_recognition_node.py
chmod +x my_wheelchair/scripts/navigation_command_node.py

# Launch multiple nodes
ros2 run my_wheelchair mapping_node &
ros2 run my_wheelchair navigation_node &
ros2 run my_wheelchair voice_recognition_node &
ros2 run my_wheelchair navigation_feedback_node &
ros2 run my_wheelchair voice_feedback_node &

# Wait for all background processes to finish
wait