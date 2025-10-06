import numpy as np
import cv2
from stitching import Stitcher
from skimage.metrics import structural_similarity as ssim

class ViewPoint():
    '''A single viewpoint.
    A View object is used to store the information of a single viewpoint.
    '''
    _ID = 0
    def __init__(self, observations:np.ndarray) -> None:
        self.id = ViewPoint._ID
        self.full_ob = observations
        # self.obs_positions = positions

        ViewPoint._ID += 1

    def update_view(self, observation:np.ndarray) -> None:
        self.full_ob = observation

class ViewMemory():
    """
    Stitch consecutive images into a single viewPoint and 
    return the matching score of given images to memorised images
    """
    def __init__(self, matching_threshold:float=0.65) -> None:
        self.views = []
        self.best_confidence_th = 0.9
        self.reset_stitcher()
        self.matching_threshold = matching_threshold
    
    def set_memory_views(self, views):
        self.views = views
    def get_memory_views(self):
        return self.views
    
    def create_viewpoint(self, observations:np.ndarray):
        '''Create a new ViewPoint and register it in the views memory

        :param observations: panorama of observations 

        '''
        viewpoint = ViewPoint(observations)
        self.views.append(viewpoint)
        return viewpoint
    
    def update_viewpoint_observation(self,view_id:int, observation:np.ndarray)-> None:
        self.views[view_id].update_view(observation)

    def stitch_images(self,images:list, conf_threshold:float=0.9) -> np.ndarray:
        if len(images) == 1:
            if images[0].shape[0]> 1000 and images[0].shape[1]> 2000:
                images[0] = cv2.resize(images[0], (int(images[0].shape[1]/4),int(images[0].shape[0]/4)))
            return images[0]
        
        if conf_threshold != self.best_confidence_th:
            self.stitcher = Stitcher(confidence_threshold=conf_threshold, detector='sift', crop=True)
        try:
            curr_ob = self.stitcher.stitch(images)
        except Exception as e: 
            print("[INFO] image stitching failed in ViewProcess:", e)
            return None
        return curr_ob      
    
    def reset_stitcher(self):
        self.stitcher = Stitcher(confidence_threshold=self.best_confidence_th, detector='sift', crop=True)

    def compare_images_SSIM(self,image1:np.ndarray, image2:np.ndarray) -> float:
        # Calculate SSIM
        #SSIM also does not perform very well on blurry or noisy images. https://stackoverflow.com/questions/11541154/checking-images-for-similarity-with-opencv
        image2 = cv2.resize(image2, (image1.shape[1], image1.shape[0]), interpolation = cv2.INTER_AREA)
        image1_gray = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        image2_gray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
        ssim_score,_ = ssim(image1_gray, image2_gray, full=True)
        
        return np.round(ssim_score,2)
    
    def compare_memorised_views_to_view(self, curr_ob:np.ndarray):
        match_scores = []
        for viewpoint in self.views:            
            score = self.compare_images_SSIM(curr_ob, viewpoint.full_ob)
            match_scores.append(score)
        return match_scores
    
    def get_closest_view_id(self, curr_ob:np.ndarray, matching_threshold:float=None):
        if matching_threshold is None:
            matching_threshold = self.matching_threshold
        match_scores = self.compare_memorised_views_to_view(curr_ob)
        if len(match_scores) == 0 or max(match_scores) <= self.matching_threshold:
            return None, match_scores
        return int(np.argmax(match_scores)), match_scores

    #same as CALL, FOR TEST PURPOSES return match_scores
    def process_image(self,images:list, conf_threshold:float=0.9):
        """ Compare new view with prev views and get the proba of each observation given data"""
              
        curr_ob = self.stitch_images(images,conf_threshold)
        if curr_ob is None: 
            return None, []
        #Get similarity score with previous views
        match_scores = self.compare_memorised_views_to_view(curr_ob)

        #If no registered views or no view match > match th prob, then we create a new observation
        if len(match_scores) == 0 or max(match_scores) <= self.matching_threshold:
            self.create_viewpoint(curr_ob)
            match_scores.append(1)
        else:
            #Replace observation if the new one is larger (more visual info)
            closest_view_id = np.argmax(match_scores)
            if curr_ob.shape[1] > self.views[closest_view_id].full_ob.shape[1]:
                self.update_viewpoint_observation(closest_view_id, curr_ob)
        
        #print('ViewMemory:match_scores current images to memorised views', match_scores)
        return int(np.argmax(match_scores)), match_scores
    
