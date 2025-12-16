from google import genai
from google.genai import types
import argparse
import cv2
from pathlib import Path
from typing import List, Optional, Dict
import numpy as np
from pydantic import BaseModel, Field
import textwrap
from PIL import Image
import json
import os

from aimapp.visualisation_tools import pickle_load_model
from pathlib import Path
import logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Suppress verbose logging from Google SDK
logging.getLogger('google').setLevel(logging.WARNING)
logging.getLogger('google.auth').setLevel(logging.WARNING)
logging.getLogger('urllib3').setLevel(logging.WARNING)

class BoundingBox(BaseModel):
    """Bounding box coordinates and dimensions."""
    xmin: int = Field(description="Left x coordinate in pixels")
    ymin: int = Field(description="Top y coordinate in pixels")
    xmax: int = Field(description="Right x coordinate in pixels")
    ymax: int = Field(description="Bottom y coordinate in pixels")


class ObjectDetectionResult(BaseModel):
    """Result of detecting an object in an image."""
    object_found: bool = Field(description="Whether the specified object was found in the image")
    confidence: float = Field(description="Confidence score between 0.0 and 1.0 for the detection")
    description: str = Field(description="Description of where/how the object appears in the image")
    bounding_box: Optional[BoundingBox] = Field(default=None, description="Bounding box around the detected object in pixels")
    reasoning: str = Field(description="Explanation of the detection")


class GeminiObjectFinder:
    """Finds specific objects in a bank of images based on text prompt."""

    def __init__(self, model_id: str = "gemini-2.0-flash-exp", temperature: float = 0.2,
                 env: str = ".google.env", api_key: Optional[str] = None):
        """
        Initialize object finder with bounding box detection.

        Args:
            model_id: Gemini model to use
            temperature: Sampling temperature
            env: Path to environment file with API key
            api_key: Direct API key (overrides env file)
        """
        self.model_id = model_id
        self.temperature = temperature

        api_key = os.environ['GEMINI_API_KEY']

        self.api_key = api_key
        self.gemini_client = genai.Client(api_key=api_key)

    def _create_detection_prompt(self, object_description: str) -> str:
        """Create prompt for object detection with bounding box."""
        return textwrap.dedent(f"""\
            You are an expert at object detection and spatial analysis in images.
            Your task is to detect the following object in the provided image:

            **TARGET OBJECT: {object_description}**

            **Critical Requirements:**
            1. **Object Detection**: Determine if the specified object is present in the image.
            2. **Bounding Box**: If found, provide a tight bounding box around the object in pixel coordinates.
               - The bounding box should tightly enclose the main body of the object
               - Use xmin, ymin (top-left corner) and xmax, ymax (bottom-right corner)
            3. **Confidence**: Provide a confidence score for the detection (0.0 to 1.0).
            4. **Description**: Describe where in the image the object appears and its characteristics.

            Analyze the image carefully and provide structured detection results with bounding box.
        """)

    def find_object_in_image(self, image, object_description: str) -> Dict:
        """
        Detect a specific object in a single image with bounding box.

        Args:
            image: Image to analyze (numpy array, PIL Image, or file path)
            object_description: Text description of the object to find

        Returns:
            Dictionary with detection results including bounding box and dimensions
        """

        request_contents = [
            image,
            self._create_detection_prompt(object_description)
        ]

        config_params = {
            "temperature": self.temperature,
            "response_mime_type": "application/json",
            "response_schema": ObjectDetectionResult,
        }

        response = self.gemini_client.models.generate_content(
            model=self.model_id,
            contents=request_contents,
            config=types.GenerateContentConfig(**config_params)
        )

        result = json.loads(response.text)

        # Calculate bounding box dimensions if available
        if result.get('bounding_box'):
            bbox = result['bounding_box']
            result['bbox_width'] = bbox['xmax'] - bbox['xmin']
            result['bbox_height'] = bbox['ymax'] - bbox['ymin']
        else:
            result['bbox_width'] = 0
            result['bbox_height'] = 0

        return result

    def find_object_in_image_bank(self,
                                  object_description: str,
                                  images: List[np.ndarray],
                                  image_ids: List[int],
                                  confidence_threshold: float = 0.5,
                                  return_all: bool = False, 
                                  logging=None) -> Dict:
        """
        Find a specific object across a list of images and identify the closest view.

        Images are ranked by:
        1. Whether object was found
        2. Detection confidence
        3. Bounding box height (larger height = closer to object)

        Args:
            object_description: Text description of the object to find (e.g., "red chair", "laptop")
            images: List of images as numpy arrays (np.ndarray)
            image_ids: List of IDs corresponding to each image
            confidence_threshold: Minimum confidence to consider a detection valid
            return_all: If True, return all detections; if False, only return images where object was found

        Returns:
            Dictionary containing:
                - 'closest_image': Info about image with closest view of the object
                - 'all_detections': List of all detection results
                - 'images_with_object': List of images where object was found
                - 'summary': Statistics about the search
        """
        if len(images) != len(image_ids):
            raise ValueError(f"Number of images ({len(images)}) must match number of IDs ({len(image_ids)})")

        if not images:
            if logging:
                logging.error("No images provided")
            return self._empty_result()
        if logging: 
            logging.info(f"\n{'='*70}")
            logging.info(f"Searching for: '{object_description}'")
            logging.info(f"Total images to scan: {len(images)}")
            logging.info(f"Ranking: Object presence > Confidence > Bounding box height")
            logging.info(f"{'='*70}\n")

        # Process each image
        all_detections = []
        images_with_object = []

        for idx, (image, image_id) in enumerate(zip(images, image_ids), 1):
            try:
                #print(f"[{idx}/{len(images)}] Analyzing image ID {image_id}...", end=" ")

                prepared_image = self._prepare_image(image)
                detection = self.find_object_in_image(prepared_image, object_description)

                # Add metadata
                detection['image_id'] = image_id
                detection['image_name'] = str(image_id)

                all_detections.append(detection)

                if detection['object_found'] and detection['confidence'] >= confidence_threshold:
                    images_with_object.append(detection)
                    bbox_info = f"bbox: {detection['bbox_width']}x{detection['bbox_height']}px"
                    #print(f"✓ FOUND (confidence: {detection['confidence']:.2%}, {bbox_info})")
                # else:
                #     print(f"✗ Not found (confidence: {detection['confidence']:.2%})")

            except Exception as e:
                logging.error(f"✗ Error: {e}")
                continue

        # Determine closest image
        closest_image = self._find_closest_image(images_with_object)

        # Prepare results
        result = {
            'closest_image': closest_image,
            'all_detections': all_detections if return_all else images_with_object,
            'images_with_object': images_with_object,
            'summary': {
                'object_searched': object_description,
                'total_images_scanned': len(images),
                'images_with_object': len(images_with_object),
                'closest_image_id': closest_image['image_id'] if closest_image else None,
                'closest_image_name': closest_image['image_name'] if closest_image else None,
                'closest_bbox_height': closest_image['bbox_height'] if closest_image else None,
            }
        }

        self._print_summary(result, logging)
        return result

    def _find_closest_image(self, detections: List[Dict],
                           bbox_weight: float = 0.5,
                           confidence_weight: float = 0.5) -> Optional[Dict]:
        """
        Find the image with the closest view of the object using comparative weighted scoring.

        Only considers images where object_found = True, then ranks them by:
        - Relative bounding box height compared to all other detections
        - Relative confidence compared to all other detections

        Score = (bbox_weight x bbox_ratio) + (confidence_weight x confidence)
        where bbox_ratio = this_image_bbox_height / max_bbox_height_across_all_images

        Args:
            detections: List of detection results
            bbox_weight: Weight for bounding box size (default 0.5)
            confidence_weight: Weight for confidence (default 0.5)

        Returns:
            Detection dict with highest weighted score, or None if no valid detections
        """
        if not detections:
            return None

        # Filter to only images where object was found
        valid_detections = [d for d in detections if d.get('object_found', False)]

        if not valid_detections:
            return None

        # Find max bbox_height across all valid detections for normalization
        max_bbox_height = max(d.get('bbox_height', 0) for d in valid_detections)

        # Avoid division by zero
        if max_bbox_height == 0:
            # If no bbox heights, sort by confidence only
            return sorted(valid_detections, key=lambda x: -x.get('confidence', 0))[0]

        # Calculate comparative weighted score for each detection
        for detection in valid_detections:
            bbox_height = detection.get('bbox_height', 0)
            confidence = detection.get('confidence', 0)

            # Normalize bbox_height relative to the maximum (0 to 1 range)
            bbox_ratio = bbox_height / max_bbox_height

            # Calculate weighted score (higher is better)
            weighted_score = (bbox_weight * bbox_ratio) + (confidence_weight * confidence)

            # Store score in detection for transparency
            detection['weighted_score'] = weighted_score
            detection['bbox_ratio'] = bbox_ratio

        # Sort by weighted score (descending - highest first)
        sorted_detections = sorted(valid_detections, key=lambda x: -x['weighted_score'])

        return sorted_detections[0]

    def _empty_result(self) -> Dict:
        """Return empty result structure."""
        return {
            'closest_image': None,
            'all_detections': [],
            'images_with_object': [],
            'summary': {
                'object_searched': None,
                'total_images_scanned': 0,
                'images_with_object': 0,
                'closest_image_name': None,
                'closest_bbox_height': None,
            }
        }

    def _print_summary(self, result: Dict, logging=None):
        """Print formatted summary of search results."""
        summary = result['summary']
        if logging:
            logging.info(f"\n{'='*70}")
            logging.info("SEARCH RESULTS SUMMARY")
            logging.info(f"{'='*70}")
            logging.info(f"Object searched:        '{summary['object_searched']}'")
            logging.info(f"Total images scanned:   {summary['total_images_scanned']}")
            logging.info(f"Images with object:     {summary['images_with_object']}")

        if result['closest_image']:
            closest = result['closest_image']
            if logging:
                logging.info(f"\n🎯 CLOSEST IMAGE FOUND:")
                logging.info(f"   Image ID:      {closest['image_id']}")
                logging.info(f"   Confidence:    {closest['confidence']:.2%}")
                logging.info(f"   Description: {closest['description']}")

            else:
                print(f"\n🎯 CLOSEST IMAGE FOUND:")
                print(f"   Image ID:      {closest['image_id']}")
                print(f"   Confidence:    {closest['confidence']:.2%}")

            if 'weighted_score' in closest and logging:
                logging.info(f"   Weighted Score: {closest['weighted_score']:.4f}")
                logging.info(f"   BBox Ratio:     {closest.get('bbox_ratio', 0):.2%}")

            if closest.get('bounding_box') and logging:
                bbox = closest['bounding_box']
                logging.info(f"   Bounding Box:")
                logging.info(f"      Position:   ({bbox['xmin']}, {bbox['ymin']}) to ({bbox['xmax']}, {bbox['ymax']})")
                logging.info(f"      Dimensions: {closest['bbox_width']} x {closest['bbox_height']} pixels (W x H)")
                logging.info(f"      Height:     {closest['bbox_height']} pixels")

        elif logging:
            logging.warning(f"\n❌ Object not found in any images")

        if len(result['images_with_object']) > 1 and logging:
            logging.info(f"\n📋 OTHER IMAGES WITH OBJECT (ranked by weighted score):")
            for i, det in enumerate(result['images_with_object'][1:], 2):
                bbox_str = f"{det['bbox_width']}x{det['bbox_height']}px"
                score_str = f"score: {det.get('weighted_score', 0):.4f}" if 'weighted_score' in det else ""
                logging.info(f"   {i}. Image ID {det['image_id']} - "
                      f"confidence: {det['confidence']:.2%}, "
                      f"bbox: {bbox_str}, {score_str}")
        if logging:
            logging.info(f"{'='*70}\n")

    def _prepare_image(self, img):
        """Convert image to PIL format."""
        if isinstance(img, Image.Image):
            pass

        # Handle file path (string or Path)
        elif isinstance(img, (str, Path)):
            img = Image.open(img)

        # Convert numpy array to PIL
        elif isinstance(img, np.ndarray):
            if len(img.shape) == 3 and img.shape[2] == 3:
                # Assume BGR format from OpenCV
                rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb_img)
            else:
                img = Image.fromarray(img)
        else:
            raise ValueError(f"Unsupported image type: {type(img)}")

        img = img.convert('RGB')
        w, h = img.size
        if w > 768:
            img = img.resize((int(w*0.4), int(h*0.4)), Image.Resampling.LANCZOS)
        return img


def gemini_determine_goal_image(imgs:list[np.ndarray], imgs_id:list[int], objective:str, confidence_threshold:float=0.7, logging=None):

    # Initialize object finder
    finder = GeminiObjectFinder(model_id='gemini-robotics-er-1.5-preview')

    # Find the object across all images
    results = finder.find_object_in_image_bank(
        object_description=objective,
        images=imgs,
        image_ids=imgs_id,
        confidence_threshold=confidence_threshold,
        return_all=False,  # Only return images where object was found
        logging= logging
    )
    if results is None:
        return None
    
    goal_image = results['closest_image']['image_id']
    return goal_image


def determine_observation_objective(model_dir:str, objective:str,confidence_threshold:float=0.7, logging=None):

    model = pickle_load_model(model_dir)
    imgs, imgs_id = extract_images_from_model(model)
    
    goal_image = gemini_determine_goal_image(imgs, imgs_id, objective, confidence_threshold,logging)
    if goal_image is None:
        raise ValueError(f"The objective {objective} was not found in any images. please retry with something else.")
    
    model_observations = model.get_A()[0]
    if goal_image > len(model_observations):
        raise ValueError(f"Observation {goal_image} is not in model!! A dimension: {len(model_observations)} ")
    likely_state = model.get_observation_most_likely_states([goal_image,-1], per_threshold=0.45)

    if len(likely_state) > 1 and logging is not None:
        logging.warning(f'WARNING: for objective {objective}, best observation is estimated to be {goal_image}, more than one state might correspond to it: {likely_state}.')
    elif len(likely_state) == 0 and logging is not None:
        logging.warning(f'WARNING: for objective {objective}, best observation is estimated to be {goal_image}. But no clear state holds that observation (catastrophic forgetting).')
    return goal_image

def extract_images_from_model(model:object):
    views = model.get_memory_views()

    small_imgs = []
    small_imgs_id = []
    for viewpoint in views:
        small_imgs_id.append(viewpoint.id)
        small_img = cv2.cvtColor(viewpoint.full_ob, cv2.COLOR_BGR2RGB)
        small_img = resize_image(small_img)
        small_imgs.append(small_img)
        
    return small_imgs, small_imgs_id

def resize_image(img, max_dimension:int=767):
    ''' make sure that the image is max max_dimension in awidth and height '''
    img_shape = img.shape
    resising_ratio = 1
    if max(img_shape) > max_dimension :
        resising_ratio = max_dimension / max(img_shape) 
    # print(resising_ratio, int(img_shape[0]*resising_ratio))
    # print(int(img_shape[0]*resising_ratio),int(img_shape[1]*resising_ratio))
    small_img = cv2.resize(img, (int(img_shape[1]*resising_ratio),int(img_shape[0]*resising_ratio)), dst=None, fx=None, fy=None, interpolation=cv2.INTER_LINEAR)
    return small_img

def save_imgs_in_folder(model_dir:str, imgs:list, imgs_id:list=None):
    """ this is just for human visual check"""
   
    model_dir_obs = Path(model_dir + '/observations') 
    model_dir_obs.mkdir(exist_ok=True, parents=True)
    print('saving observations in', model_dir_obs)
    if imgs_id is None:
        for i, img in enumerate(imgs):
            save_img(img,i, model_dir_obs)
    else:
        for i, img in zip(imgs_id,imgs):
            save_img(img,i, model_dir_obs)

def save_img(img, i:int, model_dir_obs:Path):
    img_id = str(i) + '.jpg'
    img_file = model_dir_obs  / img_id
    print('img_file:', img_file)
    cv2.imwrite(str(img_file), img)


def get_data_dir(start_folder:str=None, test_id:str=None):
    """Get test directory from test_id (same logic as action_process_no_motion.py)"""
    if start_folder is None:
        start_folder = Path.cwd()
    else:
        start_folder = Path(start_folder)
    if test_id is None:
        test_id = 0
        while os.path.isdir(start_folder / 'tests' / str(test_id)):
            test_id += 1
        store_path = start_folder / 'tests' / str(test_id - 1)
    else:
        store_path = start_folder / 'tests' / str(test_id)

    return str(store_path)


def get_goal_ob_from_model_and_gemini(test_id:str, objective:str, logging=None):
    import time
    # Convert test_id to model_dir
    model_dir = get_data_dir(None, test_id) if test_id != 'None' else None


    if len(objective) < 2 or model_dir is None:
        raise ValueError(f"The objective or model_dir is empty")

    if logging is not None:
        logging.info(f'retrieving model from {model_dir}')

    goal_image = determine_observation_objective(model_dir, objective, confidence_threshold=0.7, logging=logging)

    if logging is not None:
        logging.info(f'returned goal_image {goal_image}')

    return goal_image

def main(test_id, objective):

    # Convert test_id to model_dir
    model_dir = get_data_dir(None, test_id) if test_id != 'None' else None
    # imgs, imgs_id = extract_images_from_model(model_dir)
    # save_imgs_in_folder(model_dir, imgs, imgs_id)


    if len(objective) < 2 or model_dir is None:
        raise ValueError(f"The objective or model_dir is empty")
    
    logging.info(f'retrieving model from {model_dir}')
    goal_image =  determine_observation_objective(model_dir, objective, confidence_threshold=0.7, logging=logging)

    logging.info(f'returned goal_image {goal_image}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Reach position x,y')
    parser.add_argument('--test_id', type=str, default='None',  help='test_id (we must be in ros workspace)')
    parser.add_argument('--objective', type=str, default='',  help='object to search for')
    args = parser.parse_args()
    main(args.test_id, args.objective)