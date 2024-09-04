# Use the official ROS Humble image as a base
FROM ros:humble

# Set the working directory inside the container
WORKDIR /home/simulation/battery_ws

# Create the src directory to avoid the error
RUN mkdir -p src

# Update package list and install necessary dependencies
RUN apt-get update && \
    apt-get install -y python3-pip ros-humble-rqt-image-view

# Install Python dependencies
RUN pip3 install pydantic==1.10.12

# Run rosdep to install ROS dependencies (ensure src exists)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

