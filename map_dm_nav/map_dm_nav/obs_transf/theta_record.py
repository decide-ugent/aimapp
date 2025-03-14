import subprocess
import cv2
import os
import sys
from datetime import datetime
import time

def kill_gphoto2_processes():
    """ 
    we can't use gphoto2 if it's used by another module 
    (if the camera is mounted for visualisation for instance)
    """
    try:
        # Kill all running gphoto2 processes
        subprocess.run(["pkill", "-f", "gphoto2"], check=True, text=True)
        print("Terminated any existing gphoto2 processes.")
    except subprocess.CalledProcessError:
        # Ignore error if no gphoto2 process is running
        print("No gphoto2 processes were running.")

def capture_image(images_dir,iteration):
    """
    Capture image and save it in images_dir, if the camera is in power saving mode
    it will return an error (the command wake the camera, but too late), so we try again 
    (max 6 times) after a 1s sleep.
    """
    # Create the images directory if it doesn't exist
    
    os.makedirs(images_dir, exist_ok=True)

    # Generate a timestamped filename
    timestamp = datetime.now().strftime("IMG-%Y-%m-%d_%H-%M-%S")
    filename = f"{images_dir}/{timestamp}.jpg"

    # gphoto2 command
    command = [
        "gphoto2",
        "--capture-image-and-download",
        "--filename",
        filename
    ]
    try:
        # Run the command
        result = subprocess.run(command, check=True, text=True, capture_output=True)
        print("Image captured and saved as:", filename)
        print(result.stdout)  # Print gphoto2's output
    except subprocess.CalledProcessError as e:
        print("Error capturing image:", e.stderr)
        time.sleep(1)
        if iteration < 5:
            capture_image(images_dir, iteration=iteration+1)

def get_latest_image_file(directory):
    """ we get the latest date image from repo"""
    # Get all files in the directory
    files = [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]
    
    jpg_files = [f for f in files if f.lower().endswith(".jpg")]

    if not jpg_files:
        raise FileNotFoundError("No .jpg files found in the directory.")

    # Extract the latest file by modification time
    latest_file = max(jpg_files, key=os.path.getmtime)
    return latest_file

def read_image(file_path):
    # Read the image using cv2
    image = cv2.imread(file_path)
    if image is None:
        raise FileNotFoundError(f"Failed to read the image: {file_path}")
    return image

def show_reduced_image(image):
    print(image.shape)
    print(sys.getsizeof(image))
    image = cv2.resize(image, (int(image.shape[1]/4),int(image.shape[0]/4)))
    print(image.shape)
    print(sys.getsizeof(image))
    cv2.imshow('image window', image)
    # add wait key. window waits until user presses a key
    cv2.waitKey(0)
    # and finally destroy/close all open windows
    cv2.destroyAllWindows()
 
def main():
    print('start')
    images_dir = "/home/idlab332/theta_ricoh_x/images"
    kill_gphoto2_processes()
    capture_image(images_dir, iteration=0)

    latest_image_path = get_latest_image_file(images_dir)
    print(f"Latest image: {latest_image_path}")
    
    # Read the image
    image = read_image(latest_image_path)

    show_reduced_image(image)

    print(image.shape)


if __name__ == "__main__":
    main()
    
