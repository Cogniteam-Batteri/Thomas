# Use the official ROS Humble image as a base
FROM dustynv/ros:humble-ros-base-l4t-r36.2.0

# Set the working directory inside the container
WORKDIR /home/jetson/thomas_ws/

# Create the src directory to avoid the error
RUN mkdir -p src

# Update package list and install necessary dependencies
RUN apt-get update && \
    apt-get install -y python3-pip 

# Install Python dependencies
RUN pip3 install pydantic==1.10.12

RUN  apt install ros-humble-diagnostic-updater -y


RUN  apt install ros-humble-ackermann-msgs -y

#JOY
RUN apt-get install libcwiid1 libcwiid-dev -y
RUN apt-get install libbluetooth-dev -y
RUN  apt-get install libx11-dev -y
RUN apt-get install libspnav-dev -y
