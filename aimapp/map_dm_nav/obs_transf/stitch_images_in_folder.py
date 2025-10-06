#!/usr/bin/env python3
import os
import sys
import numpy as np
import cv2
from stitching import Stitcher
from skimage.metrics import structural_similarity as ssim


def stitch_images(images:list, conf_threshold:float=0.75) -> np.ndarray:
        if len(images) == 1:
            if images[0].shape[0]> 1000 and images[0].shape[1]> 2000:
                images[0] = cv2.resize(images[0], (int(images[0].shape[1]/4),int(images[0].shape[0]/4)))
            return images[0]

        stitcher = Stitcher(confidence_threshold=conf_threshold, detector='sift', crop=True)
        try:
            curr_ob = stitcher.stitch(images)
        except Exception as e: 
            print("[INFO] image stitching failed in ViewProcess:", e)
            return None
        return curr_ob      

def save_image_in_folder(folder_path, panorama):
    filename = os.path.join(folder_path, 'panorama.jpg')
    cv2.imwrite(filename, panorama)
    
def get_images_from_folder(folder_path):
    valid_names = {"image_0", "image_0bis", "image_1", "image_1bis", "image_2", "image_2bis"}
    images_list = []
    
    if not os.path.isdir(folder_path):
        print(f"Error: {folder_path} is not a valid directory.")
        return []
    
    for filename in sorted(os.listdir(folder_path)):
        name, ext = os.path.splitext(filename)
        
        if 'panorama' in filename:
            continue
        print('filename', filename)
        # if name in valid_names and ext.lower() in {".jpg", ".png"}:
        image_path = os.path.join(folder_path, filename)
        images_list.append(cv2.imread(image_path))
    
    return images_list

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <folder_path>")
        sys.exit(1)
    
    folder_path = sys.argv[1]
    images = get_images_from_folder(folder_path)
    panorama = stitch_images(images)
    save_image_in_folder(folder_path,panorama)
    