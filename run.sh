#!/bin/bash

IMAGE_NAME="pioneer_ros2"
PROJECT_DIR="$(pwd)"
DEVICE_PATH="/dev/ttyUSB0"

echo "🛠️  Building Docker image: $IMAGE_NAME..."
docker build --platform=linux/amd64 -t $IMAGE_NAME .

echo "🚀 Running Docker container and launching interactive shell..."
docker run -it --rm \
  --net=host \
  --privileged \
  --device=$DEVICE_PATH \
  -v "$PROJECT_DIR":/root/ROS2-Project \
  $IMAGE_NAME \
  bash -c "cd /root/ROS2-Project && bash"