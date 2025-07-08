#!/bin/bash

# Configuration
ROBOT_USER=husarion
ROBOT_IP=10.10.131.145
REMOTE_IMAGE_DIR=/home/husarion/Pictures/theta_ricoh_x
LOCAL_IMAGE_DIR="$(pwd)/husarion_photos"
FILENAME="IMG-$(date +'%Y-%m-%d_%H-%M-%S').jpg"

echo "${LOCAL_IMAGE_DIR}"
#Create local directory if it doesn't exist
mkdir -p "$LOCAL_IMAGE_DIR"

# SSH command to capture the image
ssh ${ROBOT_USER}@${ROBOT_IP} bash -c "'
    mkdir -p ${REMOTE_IMAGE_DIR}
    # Kill any leftover gphoto2 processes not attached to a terminal
    for pid in \$(pgrep gphoto2); do
        if ! ps -p \$pid -o tty= | grep -q tty; then
            kill -9 \$pid
        fi
    done
    attempt=0
    max_attempts=6
    while [ \$attempt -lt \$max_attempts ]; do
        if gphoto2 --capture-image-and-download --filename ${REMOTE_IMAGE_DIR}/${FILENAME}; then
            echo \"Image captured: ${FILENAME}\"
            break
        else
            echo \"Attempt \$((++attempt)) failed. Retrying in 1s...\"
            sleep 1
        fi
    done
'"

# Copy the image from the robot to the local machine
scp ${ROBOT_USER}@${ROBOT_IP}:${REMOTE_IMAGE_DIR}/${FILENAME} ${LOCAL_IMAGE_DIR}/

# Optional: delete the image from the robot
ssh ${ROBOT_USER}@${ROBOT_IP} "rm -f ${REMOTE_IMAGE_DIR}/${FILENAME}"

echo "Image saved locally to ${LOCAL_IMAGE_DIR}/${FILENAME}"
