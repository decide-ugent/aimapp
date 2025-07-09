#!/usr/bin/env python3
import rclpy
import sys
import os
import copy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
from cv_bridge import CvBridge
import cv2
from map_dm_nav.motion.potential_field_client import PFClient
from map_dm_nav.motion.move_straight_client import MSClient
from map_dm_nav.motion.nav2_client import Nav2Client
from map_dm_nav.obs_transf.get_pano_multiple_camera_client import PanoramaMultipleCamClient
from map_dm_nav.obs_transf.get_360_camera_client import Panorama360CamClient
from map_dm_nav.obs_transf.observation_match import ViewMemory
from map_dm_nav.model.V5 import Ours_V5_RW

#visualisations
from map_dm_nav.visualisation_tools import create_save_data_dir, save_failed_step_data, remove_white_border,\
                                                    save_step_data, save_efe_plot,pickle_load_model, pickle_dump_model, save_pose_data

os.environ["QT_QPA_PLATFORM"] = "xcb"

class HighLevelNav_ROSInterface(Node):

    def __init__(self, model_dir, goal_path):
        super().__init__('HighLevelNav_model')
        self.get_logger().info('HighLevelNav_model node has been started.')
        
        self.model_dir = model_dir
        self.goal_path = goal_path

        #dist motion in m 
        self.influence_radius = 2
        self.robot_dim = 0.25
        #The lidar must say that there is X free dist behind position to consider it free #security

        self.panorama_client = Panorama360CamClient()

  
        self.motion_client = Nav2Client()
        # self.motion_client = PFClient()
        #self.motion_client = MSClient()

        self.img_bridge = CvBridge()
        self.panorama_results = None
        self.next_possible_actions = [] #For data saving purposes
        self.model = None
        #====== VISUALISATION PARAMS =====#
        
        self.gt_odom = [0,0,0]
        self.store_dir = None
        self.start_time = time.time()
        self.execution_time = 0.0

        self.publish_believed_odom = self.create_publisher(
            msg_type=Point,
            topic="/believed_odom",
            qos_profile=5)

        
    #==== VISUALISATION CALLBACK ====#

    def save_model(self):
        ''' create map, transform to cv2, transform to ros msg, publish'''
        pickle_dump_model(self.model)
       
    #==== INITIALISATION METHODS ====#

    def initialise_model(self, n_actions:int)-> None:
        """ With first panorama.
          We setup the model and incorporates the first ghost nodes
        """
        self.Views = ViewMemory(matching_threshold=0.7) #not in model because i can't pickle it
        
        if self.model_dir == 'None':
            obstacle_dist_per_actions, ob_id, ob_match_score = self.get_panorama(n_actions)
            #create model
            self.model = Ours_V5_RW(num_obs=2, num_states=2, dim=2, \
                                    observations=[ob_id], lookahead_policy=10,\
                                    n_actions=n_actions, influence_radius=self.influence_radius,\
                                    robot_dim=self.robot_dim, lookahead_node_creation= 8)
            
            self.model.set_memory_views(self.Views.get_memory_views())
            self.model.update_transition_nodes(obstacle_dist_per_actions=obstacle_dist_per_actions)
            self.model.update_C_dim()
        #NOTE: THIS ASSUME WE RESTART FROM LAST POSITION. TO UPDATE TO RESET CURRENT POSE AND SEARCH WHEREABOUt
        else:
            #load model
            self.model = pickle_load_model(self.model_dir)
            self.Views.set_memory_views(self.model.get_memory_views())
            #self.model.policy_len = 5

            #Didn't exist during exploration runs (they were fixed parameters, not self)
            self.model.num_simulations = 30  # Number of MCTS simulations per planning step
            self.model.max_rollout_depth = 10 # Maximum depth for the simulation (rollout) phase
            self.model.c_param = 5

            obstacle_dist_per_actions, ob_id, ob_match_score = self.get_panorama(n_actions)
            self.get_logger().info('start POSE: ' + str(self.model.PoseMemory.get_odom())+', '+str(self.model.current_pose))

            #self.model.reset()
            #p_idx = self.model.infer_position_given_ob(ob_id, z_score=2)
            #self.share_believed_odom(p_idx) #send internally believed pose to /odom so it matches internal belief
        
            self.get_logger().info('ob id :'+ str(ob_id) + 'and match score: '+ str(ob_match_score))
            #self.get_logger().info('QS: ' + str(self.model.get_belief_over_states()[0])+'len '+ str(len(self.model.get_belief_over_states()[0])))
            #self.get_logger().info('POSE: ' +str(self.model.PoseMemory.get_odom())+','+str(self.model.current_pose) + ', p_idx: ' + str(p_idx))
        self.save_model()
        
        return obstacle_dist_per_actions, ob_id, ob_match_score

    def set_navigation_mode(self)->None:
        """ Check if we have a goal and if the goal is valid.
        If no (valid) goal, we explore, else we desire to reach goal with a set weight on the preference 
        """
        ob_id = None
        if self.goal_path != 'None':
            img = process_path(self.goal_path)
            if img is not None:
                img = remove_white_border(img)
                try:
                    ob_id, ob_match_score = self.Views.get_closest_view_id(img,None)
                except Exception as e:
                    self.get_logger().warning(str(e))
                if not isinstance(ob_id,int):
                    self.get_logger().info(str(ob_id) + str(type(ob_id)) + 'is not a proper integer, matching scores: '+str(ob_match_score)+', goal set aborted')
                    
        if ob_id is not None:
            self.get_logger().info('We are aiming for goal ' + str(ob_id))
            #We give the panorama id and no pose as objective
            self.model.goal_oriented_navigation([ob_id,-1], pref_weight = 10.0)
            preferred_states = np.argwhere(self.model.Cs > np.amin((self.model.Cs>0).astype(float))).flatten()
            self.get_logger().info('Prefered states ' + str(preferred_states))
        else:
            self.model.explo_oriented_navigation()
            self.get_logger().info('We are exploring aimlessly')
            
    #==== MODEL UPDATE METHOD ====#
    def model_step_process(self, action:int): # -> tuple([np.ndarray, int, list]):
        """
        we turn 360 to get panorama, 
        extract what will be the next possible actions given scan range,
        feed RGB panorama to View process and check if new ob
        Try a few time to have a good ob if necessary

        feed the ob id and next poss actions to the model to update it 
        return the observation panorama, the ob_id and the next_poss_actions
        """
        agent_possible_directions = self.model.get_possible_actions()

        obstacle_dist_per_actions,ob_id, ob_match_score = self.get_panorama(len(agent_possible_directions))
        # self.get_logger().warn("---GET PANORAMA %s seconds ---" % round(time.time() - start_time,3))
        #convert lidar scan in "can we go in that direction", considering the model directions

        if 'STAY' in agent_possible_directions.keys():
            self.next_possible_actions.append([agent_possible_directions['STAY'],1])

        start_time = time.time() #reset start time for next step
        self.model.agent_step_update(action,[ob_id, obstacle_dist_per_actions], logs=self.get_logger())
        
        self.execution_time += time.time() - start_time
        #self.get_logger().info('QS: ' + str(self.model.get_belief_over_states()[0]) + 'len '+ str(len(self.model.get_belief_over_states()[0])))
        #self.get_logger().info('POSE: ' + str(self.model.PoseMemory.get_odom())+','+str(self.model.current_pose))
        return obstacle_dist_per_actions, ob_id, ob_match_score
    
    #==== GET METHODS ====#
    def get_n_pictures(self,n_actions)->int:
        ''' how many turns to generate a panorama with 3 cameras
        we generate as many turns as we have orientations'''
        if n_actions % 2 !=0:
            n_actions -= 1

        n_turn = 2
        return n_turn, n_actions
    
    def get_current_timestep(self)->int:
        """ To keep track of where we are in process """
        return self.model.get_current_timestep()
    
    def get_panorama(self, n_directions:int):
        """ 
        Get panorama, if the image does not generate one, 
        then retry up to 5 times while reducing the confidence threshold. 
        If we failed to get a panorama after 4 tries. We raise an error
        """
        ongoing_try = 0
        ob_id = None
        conf_threshold=0.9
        n_pics, n_actions = self.get_n_pictures(n_directions)
        while not isinstance(ob_id,int) and ongoing_try < 5:
            #we turn 360deg to get a panorama 
            self.panorama_results = self.panorama_client.turn_to_get_panorama(n_turn_stops=n_pics, n_actions=n_actions) 
                                                    #how many scans/images we take in a 360deg turn
            #Convert observations into cv2 and feed them to our View model to determine if it's new
            ob = self.from_ros_to_cv2(self.panorama_results.panorama)
            try:
                ob_id, ob_match_score = self.Views.process_image(ob,conf_threshold)
            except Exception as e:
                self.get_logger().warning(str(e))
            if not isinstance(ob_id,int):
                self.get_logger().info(str(ob_id) + str(type(ob_id)) + 'is not a proper integer, retry process')
                conf_threshold-=0.07
                ongoing_try+=1
         #I just decided to init stitcher as least as possible for computation reasons. This is my solution
        if ongoing_try>0:
            self.Views.reset_stitcher()
        if not isinstance(ob_id,int):
            raise ValueError('No panorama generated, ob id is not an integer, we cannot update model.')
        if self.model is not None:
            self.model.set_memory_views(self.Views.get_memory_views())
        return self.panorama_results.pano_scan, ob_id, ob_match_score
        
    #==== MOTIONS METHODS ====#
    def define_next_objective(self, action:int, ob_id:int=None, obstacle_dist_per_actions:list=None): #-> tuple([int, dict])
        """ We define the next action and/or determine where we should go and move there.
        If we fail, we can retry a few time, deepending on the number of actions we have, 
        but if even after a few tries we can't reach any goals. We consider the robot stucks """
        data = None
        goal_reached = False
        ongoing_try = 0

        
        possible_actions = self.model.define_next_possible_actions(obstacle_dist_per_actions, restrictive=True) #Not mandatory (just to speed up process), 
        
        #could also be possible_actions = self.model.possible_actions.copy()
        possible_actions = {k: self.model.possible_actions[k] for k in possible_actions}

        max_try = len(possible_actions)-1
        while not goal_reached and ongoing_try < max_try:
            current_pose = self.model.PoseMemory.get_odom().copy()
            if action is None:
                start_time = time.time() #reset start time for next step
                actions, data = self.model.define_actions_from_MCTS_run(num_steps=1, observations=[ob_id],next_possible_actions=list(possible_actions.keys()), logging=self.get_logger(), plot_MCTS_tree=True)
                self.execution_time += time.time() - start_time
                action = actions[0]
                self.get_logger().info('next action: ' + str(action) + ', curr ob_id: '+ str(ob_id)+ \
                                        ', current pose' + str(self.model.PoseMemory.get_odom()[:2])+ 'qs' + str(self.model.qs[0].round(3)) + ' qpi '+ str(data['qpi'][0])+ ' efe '+ str(data['efe'][0]))
                
                
            ##compare current odom to desired orientation/pose to go and determine the pose to reach
            pose_goal, next_pose_id = self.model.determine_next_pose(action)
            if next_pose_id == -1 :
                self.get_logger().error('POSE GOAL ',pose_goal,' is not known by model, IT WILL FAIL')
            pose_goal = self.model.PoseMemory.id_to_pose(next_pose_id) #just to be sure we are aiming as close as possible to known state

            #if action is not an int but a pose, we convert it to action
            # if not isinstance(action, int):
            #     action = self.model.PoseMemory.from_pose_to_action(action, possible_actions)
            ##we turn and go forward considering action direction
     
            self.model.infer_pose(pose_goal)
            self.get_logger().info('step:'+ str(self.model.get_current_timestep()))
            self.get_logger().info('action: ' + str(action)+ ', pose_goal: ' + str(pose_goal) \
                                        + ', odom:'+ str(current_pose))
            #We go to that internally estimated position
            goal_reached, self.gt_odom = self.reach_position(pose_goal)#self.model.PoseMemory.get_odom())
            #If we failed to reach goal, we retry from previous odom and by inferring action.
            if not goal_reached:
                self.get_logger().info('returning to previous pose')
                elapsed_time = int(time.time() - self.start_time)

                self.model.update_B_given_unreachable_pose(pose_goal, action)
                
                save_failed_step_data(copy.deepcopy(self.model), None, np.array([0,0]), [0], list(possible_actions.keys()), \
                 [0], self.gt_odom, action_success=False, elapsed_time=elapsed_time, store_path=self.store_dir, action_select_data=data)
                # self.save_model()
                possible_actions = {key:val for key, val in possible_actions.items() if key != action} #We remove tried action from list
                self.model.PoseMemory.reset_odom(current_pose) #We reset believed odom to previous state
            
                action = None #we infer action trhis time
                ongoing_try+=1
                
                return_start_pose, self.gt_odom = self.reach_position(self.model.PoseMemory.get_odom())
                self.get_logger().info('returned to previous pose: ' + str(return_start_pose)) 
        if goal_reached:
            self.get_logger().info('pose reached, get panorama')
        else:
            raise ValueError('We could not succeed in reaching a new position in '+ str(max_try)+' tries, robot stucked.')
        return action, data

    def reach_position(self,goal_pose:list): #-> tuple([Bool,Odometry])
        goal_reached, pose= self.motion_client.go_to_pose(goal_pose)
        if not goal_reached and pose is not None: 
            #if we are one third near the goal, let's say it's ok
            if np.allclose(pose[:2], goal_pose[:2], atol=self.influence_radius/3):
                goal_reached = True
                self.get_logger().info('Goal ' + str(goal_pose) + \
                            'reached with tolerance: ' + str(self.influence_radius/3)+'m, ended PF at pose '+ str(pose))
            else:
                self.get_logger().info('Goal ' + str(goal_pose) + \
                            ' not reached, ended PF at pose '+ str(pose))
            
        return goal_reached, pose
    
    #==== IMG METHODS ====#
    def from_ros_to_cv2(self,ros_imgs:list)->list:
        cv2_imgs = []
        for img in ros_imgs:
            cv2_imgs.append(self.img_bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")) #or "rgb8"?
        return cv2_imgs
    
    #==== SHARE BELIEF METHOD ====#
    def share_believed_odom(self, p_idx=int):
        """
        If the agent is sure above z_score of its current state, 
        we publish the believed pose so that /odom can correct itself and consider this new pose as reference
        """
        if p_idx >=0:
            current_pose = self.model.PoseMemory.id_to_pose(p_idx)
            pose = Point(x=current_pose[0], y=current_pose[1])
            self.get_logger().info('certainty about the state at step %s, believed pose: %s' % (str(self.model.get_current_timestep()), str(current_pose)))         
            self.publish_believed_odom.publish(pose)

def save_data_process(highlevelnav:object, ob_id:int, ob_match_score:list,\
                      obstacle_dist_per_actions:list, store_dir, data:dict=None):

    ob = highlevelnav.Views.views[ob_id].full_ob
    elapsed_time = int(time.time() - highlevelnav.start_time)
    highlevelnav.save_model()
    save_step_data(highlevelnav.model, ob_id, ob, ob_match_score, obstacle_dist_per_actions,\
                highlevelnav.gt_odom, action_success=True, elapsed_time=elapsed_time,\
                      store_path=store_dir, action_select_data=data, execution_time = highlevelnav.execution_time )
    if data is not None and 'poses_efe' in data:
        save_efe_plot(data['poses_efe'],highlevelnav.get_current_timestep(),store_dir)

def process_path(goal_path: str):
    """ extract image from goal path"""
    # Check if the path exists and is a file
    if not os.path.exists(goal_path):
        print("The specified path does not exist.")
        return None
    elif not os.path.isfile(goal_path):
        print("The specified path is not a file.")
        return None

    image = cv2.imread(goal_path)
    
    if image is None:
        print("Failed to load the goal image. Check if the file is a valid image format.")
        return None
    return image

def main(args=None):
    rclpy.init(args=args)
    model_dir = sys.argv[2] if len(sys.argv) > 2 else 'None'
    goal_path = sys.argv[4] if len(sys.argv) > 4 else 'None'
    
    highlevelnav = HighLevelNav_ROSInterface(model_dir, goal_path)
   
    store_dir = create_save_data_dir()
    highlevelnav.store_dir = store_dir

    
    """
    ** Initialise MODEL **
    """

    possible_actions = 13 # circle / 12 + STAY action
    policy = [None] * 200
    #policy = [(1,0),(1,1),(0,1), (0,0)]
    #policy = [6, 5, 6, 0, 3, 5, 0, 1, 4, 4, 7, 4, 2, 3, 0, 6, 8, 5, 4, 4, 2, 1, 1, 4, 1, 4, 8] #an action is a direction to take in global coordinate (0:0degree in GP)
    
    #I'm damn sick and just want to have this done with. 
    # So yes it is not elegant but i don't care anymore. I just want to send the initial pose and be done and sleep 
    #TODO: fix my mess once i get a brain
    #This part exists to send an initial pose if we are using nav2
    #We need to have the node spin to receive the odometry to send as initial pose. 
    # And we can't have the set_initial_pose called too early else it will not publish. Don't ask me why, ask my pillow
    motion_initial_pose = getattr(highlevelnav.motion_client, "set_initial_pose", None)
    if callable(motion_initial_pose):
        rclpy.spin_once(highlevelnav.motion_client, timeout_sec=2)
        highlevelnav.motion_client.set_initial_pose()


    obstacle_dist_per_actions,ob_id, ob_match_score = highlevelnav.initialise_model(possible_actions)

    highlevelnav.set_navigation_mode()
    save_data_process(highlevelnav, ob_id=ob_id, ob_match_score= ob_match_score, obstacle_dist_per_actions= obstacle_dist_per_actions, store_dir=store_dir)
    
    """
    ** RUN MODEL **
    """

    for action in policy:
        highlevelnav.execution_time = 0
        action, action_data = highlevelnav.define_next_objective(action, ob_id, obstacle_dist_per_actions)
        
        #highlevelnav.get_logger().info('checking the action %f, %s' % (action, str(type(action))))
        #NOTE: UPDATE MODEL INTERNAL ACTION MANUALLY WHEN GIVEN A STATIC POLICY
        if action_data is None:
            highlevelnav.model.set_action_step(action)
        
        obstacle_dist_per_actions, ob_id, ob_match_score = highlevelnav.model_step_process(action)

        highlevelnav.get_logger().info('qs: ' +str(highlevelnav.model.get_belief_over_states()[0].round(3)))
        p_idx = highlevelnav.model.infer_current_most_likely_pose(observations= [ob_id], z_score=10)
        highlevelnav.get_logger().info('POSE: ' +str(highlevelnav.model.PoseMemory.get_odom())+','+str(highlevelnav.model.current_pose) + 'p_idx: ' + str(p_idx))
        
        highlevelnav.share_believed_odom(p_idx) #send internally believed pose to /odom so it matches internal belief
        save_data_process(highlevelnav, ob_id, ob_match_score, obstacle_dist_per_actions=obstacle_dist_per_actions, store_dir= store_dir, data= action_data)
        #highlevelnav.get_logger().warn("---SAVE DATA PROCESS %s seconds ---" % round(time.time() - start_time,3))

        # ob = highlevelnav.Views.views[ob_id].full_ob
        # save_pose_data(highlevelnav.model, ob, ob_id, obstacle_dist_per_actions, logs=highlevelnav.get_logger())
        # plot_panorama_memories_and_odom(highlevelnav)
        highlevelnav.get_logger().info('Next action')
        
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    highlevelnav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
