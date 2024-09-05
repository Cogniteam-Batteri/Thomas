#!/bin/bash

# Set variables
IMAGE_NAME="thomas"   # Change this to your desired image name
IMAGE_TAG="humble"             # You can change this tag or make it dynamic

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Docker is not installed. Please install Docker first."
    exit 1
fi

# Build the Docker image
echo "Building Docker image: ${IMAGE_NAME}:${IMAGE_TAG}..."
docker build -t ${IMAGE_NAME}:${IMAGE_TAG} .

# Check if the build was successful
if [ $? -eq 0 ]; then
    echo "Docker image built successfully: ${IMAGE_NAME}:${IMAGE_TAG}"
else
    echo "Error: Failed to build Docker image."
    exit 1
fi
