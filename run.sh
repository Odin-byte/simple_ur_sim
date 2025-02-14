#!/bin/bash

# Set default values if not provided
IMAGE_NAME=${1:-ur5e_mujoco}
CONTAINER_NAME=${2:-ur5e_sim}
CONTAINER_WORKSPACE_DIR=${6:-"/ur5e_sim"}  # Default container workspace directory

# Enable X11 for GUI applications
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

# Create Xauthority file if it doesn't exist
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth_list=$(xauth nlist $DISPLAY)
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    fi
    chmod 777 $XAUTH
fi

# Notify the user
echo "Docker container '$CONTAINER_NAME' starting."

# Run the Docker container
docker run -it --rm \
    --name $CONTAINER_NAME \
    --net=host \
    --env DISPLAY=$DISPLAY \
    --env XAUTHORITY=$XAUTH \
    --volume $XSOCK:$XSOCK:rw \
    --volume $XAUTH:$XAUTH:rw \
    --volume ./src:/ur_sim/src:rw \
    --privileged \
    $IMAGE_NAME
