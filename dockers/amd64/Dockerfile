# Use the official ROS Humble image as a base
FROM ros:humble

# Set the working directory inside the container
WORKDIR /home/simulation/battery_ws

# Create the src directory to avoid the error
RUN mkdir -p src

# Update package list and install necessary dependencies
RUN apt-get update 

RUN apt-get update && \
    apt-get install -y python3-pip 
    
# Install Python dependencies
RUN pip3 install pydantic==1.10.12

RUN pip3 install pyserial



