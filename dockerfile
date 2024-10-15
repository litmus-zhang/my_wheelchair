FROM ros:foxy

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get install python3-pip 
RUN apt-get install portaudio19-dev
RUN pip3 install SpeechRecognition pyaudio pyttsx3 RPi.GPIO

# Set up your ROS 2 workspace
WORKDIR /ros2_ws
COPY . /ros2_ws/src/my_wheelchair

# Build your package
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --packages-select my_wheelchair

# Source the workspace in .bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Copy the startup script
COPY start_nodes.sh /start_nodes.sh
RUN chmod +x /start_nodes.sh

# Set the startup script as the entry point
ENTRYPOINT ["/start_nodes.sh"]