#!/bin/bash

# Set default image name if not provided
IMAGE_NAME=${1:-ur5e_mujoco}

# Build the Docker image
docker build -t $IMAGE_NAME .

# Notify the user
echo "Docker image '$IMAGE_NAME' built successfully."
