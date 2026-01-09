#!/bin/bash

# Configuration
ROBOT_USER=husarion
ROBOT_IP=192.168.77.2
REMOTE_IMAGE_DIR=/home/husarion/Pictures/theta_ricoh_x
LOCAL_IMAGE_DIR="$(pwd)/husarion_photos"
FILENAME="IMG-$(date +'%Y-%m-%d_%H-%M-%S').jpg"

# Enable error handling
set -e  # Exit on any error
trap 'echo "Error on line $LINENO. Command: $BASH_COMMAND"' ERR

echo "Local directory: ${LOCAL_IMAGE_DIR}"
echo "Remote robot: ${ROBOT_USER}@${ROBOT_IP}"
echo "Filename: ${FILENAME}"
echo ""

# Create local directory if it doesn't exist
mkdir -p "$LOCAL_IMAGE_DIR"

# Check if robot is reachable
echo "Checking robot connectivity..."
if ! ping -c 1 -W 2 ${ROBOT_IP} &> /dev/null; then
    echo "ERROR: Robot at ${ROBOT_IP} is not reachable!"
    echo "Please check:"
    echo "  1. Robot is powered on"
    echo "  2. Network connection is active"
    echo "  3. IP address is correct (current: ${ROBOT_IP})"
    exit 1
fi
echo "✓ Robot is reachable"

# SSH command to capture the image
echo "Capturing image on robot..."
if ! ssh ${ROBOT_USER}@${ROBOT_IP} bash -c "'
    mkdir -p ${REMOTE_IMAGE_DIR}
    # Kill any leftover gphoto2 processes not attached to a terminal
    for pid in \$(pgrep gphoto2 2>/dev/null); do
        if ! ps -p \$pid -o tty= 2>/dev/null | grep -q tty; then
            kill -9 \$pid 2>/dev/null
        fi
    done
    attempt=0
    max_attempts=6
    while [ \$attempt -lt \$max_attempts ]; do
        if gphoto2 --capture-image-and-download --filename ${REMOTE_IMAGE_DIR}/${FILENAME} 2>&1; then
            echo \"Image captured: ${FILENAME}\"
            exit 0
        else
            echo \"Attempt \$((++attempt)) failed. Retrying in 1s...\"
            sleep 1
        fi
    done
    echo \"Failed to capture image after \$max_attempts attempts\"
    exit 1
'"; then
    echo "ERROR: Image capture failed on robot"
    exit 1
fi
echo "✓ Image captured on robot"

# Copy the image from the robot to the local machine
echo "Copying image from robot to local machine..."
if ! scp -q ${ROBOT_USER}@${ROBOT_IP}:${REMOTE_IMAGE_DIR}/${FILENAME} ${LOCAL_IMAGE_DIR}/; then
    echo "ERROR: Failed to copy image from robot"
    exit 1
fi
echo "✓ Image copied to local machine"

# Verify the file exists locally
if [ ! -f "${LOCAL_IMAGE_DIR}/${FILENAME}" ]; then
    echo "ERROR: Image file not found locally: ${LOCAL_IMAGE_DIR}/${FILENAME}"
    exit 1
fi

# Delete the image from the robot
echo "Cleaning up remote image..."
if ! ssh ${ROBOT_USER}@${ROBOT_IP} "rm -f ${REMOTE_IMAGE_DIR}/${FILENAME}"; then
    echo "WARNING: Failed to delete remote image (file may still exist on robot)"
else
    echo "✓ Remote image deleted"
fi

echo ""
echo "SUCCESS: Image saved locally to ${LOCAL_IMAGE_DIR}/${FILENAME}"
ls -lh "${LOCAL_IMAGE_DIR}/${FILENAME}"
