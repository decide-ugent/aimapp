#!/usr/bin/env python3
import os
import sys
import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from pathlib import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge

from map_dm_nav.visualisation_tools import pickle_load_model, create_save_data_dir, pickle_dump_model, save_step_data,save_failed_step_data, remove_white_border
from map_dm_nav_actions.action import AIFProcess  # Custom action
from map_dm_nav.obs_transf.observation_match import ViewMemory
from map_dm_nav.obs_transf.get_360_camera_client import Panorama360CamClient
from map_dm_nav.model.V5 import Ours_V5_RW
import time
import numpy as np
import cv2

class AIFProcessServer(Node):

    def __init__(self, test_id:int='None'):
        super().__init__('aif_process')

        self.model = None
        self.robot_pose = []
        
        self.influence_radius = 3
        self.robot_dim = 0.3
        self.poses = []
        self.img_bridge = CvBridge()
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/shifted',
            self.odom_callback,
            10
        )

        self.goal_pub = self.create_publisher(
            Marker,
            '/goal_pose_marker',
            qos_policy
        )

        # self.goal_sub = self.create_subscription(
        #     Bool,
        #     '/goal_reached',
        #     self.goal_reached_callback,
        #     10
        # )

        # self.odom_sub = self.create_subscription(
        #     LaserScan,
        #     '/agent/scan',
        #     self.lidar_callback,
        #     10
        # )
        self.model_odom_pub = self.create_publisher(
            Odometry,
            '/shifted_odom',
            qos_profile=qos_policy
        )
        self.actions_pub = {}
        for i in range(13):
            self.actions_pub[i] = self.create_publisher(
                msg_type=PoseStamped,
                topic="action"+str(i),
                qos_profile=qos_policy)

        self.Views = ViewMemory(matching_threshold=0.7)
        self.panorama_client = Panorama360CamClient()
        self.start_time = time.time()
        self.execution_time = 0.0


        self.goal_path = ''
     
        if test_id == 'None':
            self.test_folder = create_save_data_dir()
            obstacle_dist_per_actions, ob_id, ob_match_score = self.initialise_model(n_actions=13)
            self.last_ob_id = ob_id
            self.prev_scans_dist = obstacle_dist_per_actions

        else:
            self.test_folder = get_data_dir(None, test_id)
            self.load_latest_model()
            self.model.reset(start_pose=(0.0,0.0))
            self.model.PoseMemory.reset_odom([0.0,0.0])
            obstacle_dist_per_actions, ob_id, ob_match_score = self.get_panorama(len(self.model.get_possible_actions()))
            # qs = self.model.get_belief_over_states()
            # qo = self.get_expected_observation(qs)
            # self.last_ob_id = np.argmax(qo[0]) #might be imbricated list. to check
            self.last_ob_id = ob_id
            self.prev_scans_dist = obstacle_dist_per_actions
            self.set_navigation_mode()
            self.save_new_model_next_step()
            


        next_possible_actions = self.model.define_next_possible_actions(obstacle_dist_per_actions, restrictive=True,logs=self.get_logger())

        ideal_next_action, data = self.model.define_actions_from_MCTS_run(num_steps=1, observations=[self.last_ob_id],next_possible_actions=next_possible_actions, save_action_memory = False, logging= self.get_logger())

        self.get_logger().info(f'THE IDEAL NEXT MOTION (FOR MCTS):{ideal_next_action}')
        
        reachable_points = []
        self.get_logger().info('AIF setup at path %s' % self.test_folder)
        for a in next_possible_actions:
            next_pose, next_pose_id = self.model.determine_next_pose(a)
            pt = Point()
            next_pose[0] = float(next_pose[0])
            next_pose[1] = float(next_pose[1])
            pt.x = next_pose[0]
            pt.y = next_pose[1]
            reachable_points.append(pt)
            self.publish_vector(next_pose[0], next_pose[1], self.actions_pub[a])
            self.get_logger().info('possible action %d poses %s' % (a, str(next_pose)))
            if a == ideal_next_action[0]:
                #GOAL POSE
                self.pub_goal_pose(next_pose)
        
        self.action_server = ActionServer(
            self,
            AIFProcess,
            'aif_process',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )

        
        self.get_logger().info('action server aif_process setup')

    def goal_cb(self, goal_request):
        self.get_logger().info('Goal request received.')
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info('Goal cancel request received.')
        return CancelResponse.ACCEPT

    def odom_callback(self, msg):
        if len(self.robot_pose) == 0:
            self.get_logger().info('Got odom.')
        self.robot_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

    def goal_reached_callback(self, msg):
        self.goal_reached = msg.data
    
    def pub_goal_pose(self, pose):
        """ Publish goal pose to the goal publisher """
        goal_pose = Marker()
        goal_pose.header.frame_id = 'map'
        goal_pose.type = Marker.SPHERE
        goal_pose.action = Marker.ADD
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.scale.x = 0.2
        goal_pose.scale.y = 0.2
        goal_pose.scale.z = 0.2
        goal_pose.color.a = 1.0
        goal_pose.color.r = 0.0
        goal_pose.color.g = 1.0
        goal_pose.color.b = 0.0
        self.get_logger().info(f'Publishing goal pose at {pose}')
        self.goal_pub.publish(goal_pose)


    def initialise_model(self, n_actions:int)-> None:
        """ With first panorama.
          We setup the model and incorporates the first ghost nodes
        """           
       
        #TODO: UPDATE PANORAMA TO KNOW ACTION RANGE
        obstacle_dist_per_actions, ob_id, ob_match_score = self.get_panorama(n_actions)
        #create model
        self.model = Ours_V5_RW(num_obs=2, num_states=2, dim=2, \
                                observations=[ob_id], \
                                n_actions=13, influence_radius=self.influence_radius,\
                                robot_dim=self.robot_dim, lookahead_node_creation= 8)
        
        self.model.set_memory_views(self.Views.get_memory_views())
        self.model.update_transition_nodes(obstacle_dist_per_actions=obstacle_dist_per_actions, logs=self.get_logger())
        self.model.update_C_dim()
        self.save_model(self.test_folder)
        self.save_data_process(ob_id, ob_match_score,\
                      obstacle_dist_per_actions, None)
        

        return obstacle_dist_per_actions, ob_id, ob_match_score
    
    def get_model_step_folder(self):
        step_id = 0 
        step = 'step_'+ str(step_id)
        step_miss = 'step_'+ str(step_id) + '*'
        while os.path.isdir( self.test_folder / step ) or os.path.isdir(self.test_folder / step_miss ):
            step_id+=1
            step = 'step_'+ str(step_id)
            step_miss = 'step_'+ str(step_id) + '*'
        step = 'step_'+ str(step_id-1)
        step_miss = 'step_'+ str(step_id-1) + '*'
        if os.path.isdir( self.test_folder / step ): 
            self.current_step_path = self.test_folder / step
        else:
            self.current_step_path = self.test_folder / step_miss
        return self.current_step_path

    def save_new_model_next_step(self):
        step_id = 0 
        step = 'step_'+ str(step_id)
        step_miss = 'step_'+ str(step_id) + '*'
        while os.path.isdir( self.test_folder / step ) or os.path.isdir(self.test_folder / step_miss ):
            step_id+=1
            step = 'step_'+ str(step_id)
            step_miss = 'step_'+ str(step_id) + '*'
        step = 'step_'+ str(step_id)
        current_step_path = self.test_folder / step
        self.get_logger().info(f'Saving new model as a new step: {current_step_path}')
        Path(current_step_path).mkdir(exist_ok=True, parents=True)
        self.save_model(current_step_path)
        
        return current_step_path


    def load_latest_model(self):
        step_folder = self.get_model_step_folder()
        self.get_logger().info(f'Extracting model from {step_folder}')
        self.model = pickle_load_model(step_folder)
        self.Views.set_memory_views(self.model.get_memory_views())

    def execute_cb(self, goal_handle):
        self.get_logger().info('Executing goal...')

        goal_action = goal_handle.request.action
        goal_success = goal_handle.request.goal_reached
        result = AIFProcess.Result()

        self.load_latest_model()
        self.execution_time = 0.0
        start_execution_time = time.time()
        next_pose, next_pose_id = self.model.determine_next_pose(goal_action)
        
        if self.model is None :
            self.get_logger().error('Model or robot pose is not ready.')
            return result
        
        if goal_success is False:
            self.model.step_time()
            self.model.update_B_given_unreachable_pose(next_pose, goal_action) 
            possible_actions = self.model.define_next_possible_actions(self.prev_scans_dist, restrictive=True, logs=self.get_logger())
            possible_actions.remove(goal_action)
            ideal_next_action, data = self.model.define_actions_from_MCTS_run(num_steps=1, observations=[self.last_ob_id],next_possible_actions=possible_actions, save_action_memory = False, logging= self.get_logger())
            elasped_time = time.time() - self.start_time
            self.save_model(self.test_folder)
            save_failed_step_data(copy.deepcopy(self.model), self.last_ob_id, np.array([0,0]), [0], possible_actions, self.prev_scans_dist, self.robot_pose, action_success=False, elapsed_time=elasped_time, store_path=self.test_folder, action_select_data=data)

            self.get_logger().info(f'THE IDEAL NEXT MOTION (FOR MCTS):{ideal_next_action}')

            result.possible_actions = possible_actions
            result.reachable_goals = [Point()]
            #GOAL POSE
            next_pose, next_pose_id = self.model.determine_next_pose(ideal_next_action[0])
            self.pub_goal_pose(next_pose)


            goal_handle.succeed()
            
            return result

        self.model.set_action_step(goal_action) #step_time()
        self.model.set_current_pose(next_pose_id)
        self.publish_odom(self.model.current_pose)
        self.execution_time = time.time() - start_execution_time
        self.get_logger().info(f'current pose {str(next_pose)}, {self.model.current_pose}, {next_pose_id}, reached with action {goal_action}')
        # Perform a model step (assume action = 0 for now)
        try:
            #also add time in model update (but not panorama taking as its sensor dependent )
            obstacle_dist_per_actions, ob_id, ob_match_score = self.model_step_process(action=goal_action, pose_id=next_pose_id)
        except Exception as e:
            self.get_logger().error(f'Failed model step: {str(e)}')
            return result
        
        start_execution_time = time.time()
        next_possible_actions = self.model.define_next_possible_actions(obstacle_dist_per_actions, restrictive=True,logs=self.get_logger())
        ideal_next_action, data = self.model.define_actions_from_MCTS_run(num_steps=1, observations=[ob_id],next_possible_actions=next_possible_actions, save_action_memory = False, logging= self.get_logger())

        self.execution_time += time.time() - start_execution_time
        self.get_logger().info(f'THE IDEAL NEXT MOTION (FOR MCTS):{ideal_next_action}')
        elasped_time = time.time() - self.start_time
        self.save_data_process(ob_id, ob_match_score, obstacle_dist_per_actions=obstacle_dist_per_actions, data= data)

        self.prev_scans_dist = obstacle_dist_per_actions
        self.last_ob_id = ob_id

        reachable_points = []
        for a in next_possible_actions:
            next_pose, next_pose_id = self.model.determine_next_pose(a)
            pt = Point()
            pt.x = next_pose[0]
            pt.y = next_pose[1]
            reachable_points.append(pt)
            self.publish_vector(next_pose[0], next_pose[1], self.actions_pub[a])
            self.get_logger().info('possible action %d poses %s' % (a, str(next_pose)))
            if a == ideal_next_action[0]:
                self.pub_goal_pose(next_pose)
                # self.get_logger().info(f'Publishing goal pose {next_pose} for action {a}')
    

        result.possible_actions = next_possible_actions
        result.reachable_goals = reachable_points
        goal_handle.succeed()
        self.get_logger().info(f'Returning {len(reachable_points)} reachable poses.')
        return result

    def publish_odom(self, pose):
        """ Publish the current pose as odometry """
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'odom'
        odom_msg.pose.pose.position.x = pose[0]
        odom_msg.pose.pose.position.y = pose[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        self.model_odom_pub.publish(odom_msg)

    def save_model(self, path):
        ''' create map, transform to cv2, transform to ros msg, publish'''
        pickle_dump_model(self.model, path)

    def save_data_process(self, ob_id:int, ob_match_score:list,\
                      obstacle_dist_per_actions:list, data:dict=None):

        ob = self.Views.views[ob_id].full_ob
        elapsed_time = 0
        if self.robot_pose is not None and len(self.robot_pose) >= 2 :
            robot_pose = self.robot_pose
        else:
            robot_pose = self.model.current_pose
    
        self.save_model(self.test_folder)
        save_step_data(self.model, ob_id, ob, ob_match_score, obstacle_dist_per_actions,\
                    robot_pose, action_success=True, elapsed_time=elapsed_time,\
                        store_path=self.test_folder, action_select_data=data, execution_time=self.execution_time)
        # if data is not None and 'poses_efe' in data:
        #     save_efe_plot(data['poses_efe'],self.model.get_current_timestep(),store_dir)

    def model_step_process(self, action: int, pose_id:int=None):
        agent_possible_directions = self.model.get_possible_actions()

        obstacle_dist_per_actions, ob_id, ob_match_score = self.get_panorama(len(agent_possible_directions))

        start_time = time.time()
        if 'STAY' in agent_possible_directions:
            self.model.next_possible_actions.append([agent_possible_directions['STAY'], 1])

        self.model.agent_step_update(action, [ob_id, pose_id, obstacle_dist_per_actions], logs=self.get_logger())
        self.execution_time += time.time() - start_time
        return obstacle_dist_per_actions, ob_id, ob_match_score

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

    def from_ros_to_cv2(self, ros_imgs):
        return [self.img_bridge.imgmsg_to_cv2(img, desired_encoding="bgr8") for img in ros_imgs]

    def get_n_pictures(self, n_actions):
        if n_actions % 2 != 0:
            n_actions -= 1
        return 2, n_actions

    def publish_vector(self,x,y, name_pub):
        """" Visualise vectors in rviz """
        vector = PoseStamped()
        vector.header.frame_id = "odom"
        vector.pose.position.x = x #max(min(x, 5.0), -5.0)
        vector.pose.position.y = y #max(min(x, 5.0), -5.0)
        vector.pose.position.z = 0.0

        yaw = np.arctan2(y,x)
        [x,y,z,w] = quaternion_from_euler(0,0,yaw)
        vector.pose.orientation.x = x
        vector.pose.orientation.y = y
        vector.pose.orientation.z = z
        vector.pose.orientation.w = w

        name_pub.publish(vector)
        return
    
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

def get_data_dir(start_folder:str=None, test_id:int=None):
        """ get latest created test directory """
        if start_folder is None:
            start_folder = Path.cwd()
        else:
            start_folder = Path(start_folder)
        if test_id is None:
            test_id = 0
            while os.path.isdir( start_folder / 'tests' / str(test_id)):
                test_id+=1
            store_path = start_folder / 'tests' / str(test_id-1)
        else:
            store_path = start_folder/ 'tests' / str(test_id)
        
        return store_path

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    return [x,y,z,w]

def main(args=None):
    rclpy.init(args=args)
    test_id = sys.argv[2] if len(sys.argv) > 2 else 'None'
    node = AIFProcessServer(test_id)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
