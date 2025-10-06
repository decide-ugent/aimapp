#!/bin/bash

xhost +local:root

# Path to the folder on your desktop (we expect it to be where you started the dockerfile )
DESKTOP_RESULTS_FOLDER="$(pwd)"

# Run the Docker container with GUI support (works on CPU here, not NVIDIA)
docker run -it --rm --privileged --net=host \
    --name aimapp \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTHORITY \
    -v ${DESKTOP_RESULTS_FOLDER}:/home/ros2_ws/src/aimapp \
    -v ${DESKTOP_RESULTS_FOLDER}/tests:/home/ros2_ws/tests \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -w /home/ros2_ws aimapp:latest

docker exec -itd aimapp bash
