#!/usr/bin/env python3
import os
import sys
import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse, ActionClient
from pathlib import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Int16, Int32MultiArray, Float64MultiArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
# from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from aimapp.visualisation_tools import pickle_load_model, create_save_data_dir, pickle_dump_model, save_step_data,save_failed_step_data, remove_white_border
from aimapp_actions.action import AIFProcess  # Custom action
from aimapp_actions.msg import NavigationResult
from aimapp.obs_transf.observation_match import ViewMemory
from aimapp.obs_transf.get_360_camera_client import Panorama360CamClient
from aimapp.model.V5 import Ours_V5_RW
from aimapp.mcts_reward_visualiser import MCTSRewardVisualiser
import time
import numpy as np
import cv2

class AIFProcessServer(Node):

    def __init__(self, test_id:int='None', goal_ob_id:int=-1, goal_pose_id:int=-1, start_node_id:int=-1, influence_radius:float=1.6, n_actions:int=17, lookahead_node_creation:int=8, skip_double_check_visited_state:bool=False):
        super().__init__('aif_process')

        self.model = None
        self.robot_pose = []
        self.latest_map = None  # Store the latest map from /map topic

        self.influence_radius = influence_radius
        self.n_actions = n_actions
        self.lookahead_node_creation = lookahead_node_creation
        self.robot_dim = 0.25
        self.model_imagine_next_action = True

        self.policy_length = 1
        self.skip_double_check_visited_state = skip_double_check_visited_state

        # Store goal IDs
        self.goal_id = [goal_ob_id, goal_pose_id]
        self.start_node_id = start_node_id
        #self.get_logger().info(f'test_id {test_id}, goal_ob_id {goal_ob_id}, goal_pose_id {goal_pose_id}, start_node_id {start_node_id}, influence_radius {influence_radius}')
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

        # Subscribe to map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_policy
        )

        

        self.goal_pub = self.create_publisher(
            Marker,
            '/goal_pose_marker',
            qos_policy
        )

        self.executed_action_pub = self.create_publisher(
            Int16,
            '/executed_action',
            qos_policy
        )

        # Publishers for GUI integration - publish possible actions and nodes
        self.possible_actions_pub = self.create_publisher(
            Int32MultiArray,
            '/aif_possible_actions',
            qos_policy
        )

        self.possible_nodes_pub = self.create_publisher(
            Int32MultiArray,
            '/aif_possible_nodes',
            qos_policy
        )

        self.reachable_goals_pub = self.create_publisher(
            Float64MultiArray,
            '/aif_reachable_goals',
            qos_policy
        )

        self.model_odom_pub = self.create_publisher(
            Odometry,
            '/shifted_odom',
            qos_profile=qos_policy
        )
        self.actions_pub = {}
        for i in range(self.n_actions):
            self.actions_pub[i] = self.create_publisher(
                msg_type=PoseStamped,
                topic="action"+str(i),
                qos_profile=qos_policy)

        self.Views = ViewMemory(matching_threshold=0.7)
        self.panorama_client = Panorama360CamClient()
        self.start_time = time.time()
        self.execution_time = 0.0

        # Store current possible actions and goals for navigation result matching
        self.current_possible_actions = []
        self.current_possible_nodes = []
        self.current_reachable_goals = []

        # Subscribe to navigation results
        self.nav_result_sub = self.create_subscription(
            NavigationResult,
            '/nav2_navigation_result',
            self.navigation_result_callback,
            qos_policy
        )

        # Action client to trigger AIFProcess action
        self.aif_action_client = ActionClient(self, AIFProcess, 'aif_process')

        # Initialize MCTS reward visualizer (will be set after model is initialized)
        self.mcts_visualizer = None

        self.goal_path = ''
     
        if test_id == 'None':
            self.test_folder = create_save_data_dir()
            obstacle_dist_per_actions, ob_id, ob_match_score = self.initialise_model(n_actions=self.n_actions)
            self.last_ob_id = ob_id
            self.prev_scans_dist = obstacle_dist_per_actions
            

        else:
            # Load existing model from test_id
            old_test_folder = get_data_dir(None, test_id)
            self.get_logger().info(f'Loading model from {old_test_folder}')

            # Check if we're in goal-reaching mode (at least one goal specified)
            is_goal_reaching_mode = (goal_ob_id >= 0) or (goal_pose_id >= 0)

            if is_goal_reaching_mode:
                # Create a new test folder for goal-reaching
                self.test_folder = create_save_data_dir()
                self.get_logger().info(f'Goal-reaching mode: Created new test folder {self.test_folder}')

                # Load model from old folder
                self.get_logger().info(f'Loading model from previous test {old_test_folder}')
                self.model = pickle_load_model(old_test_folder)
                self.Views.set_memory_views(self.model.get_memory_views())
            else:
                # Exploration mode: continue in same folder
                self.test_folder = old_test_folder
                self.load_latest_model()
                self.get_logger().info(f'Exploration mode: Continuing in test folder {self.test_folder}')

            # Reset model to start_node_id if provided
            if self.start_node_id >= 0:
                self.get_logger().info(f'Resetting model to start node {self.start_node_id}')

                # Extract pose from node ID
                start_pose = self.model.PoseMemory.id_to_pose(self.start_node_id)
                self.get_logger().info(f'Start pose from node {self.start_node_id}: {start_pose}')

                # Reset model to this pose
                self.model.reset(start_pose)

                # Update self.qs with node_id - all zeros except at start_node_id index
                import aimapp.model.pymdp.utils as utils
                self.model.qs = utils.obj_array_uniform(self.model.num_states)
                for i in range(len(self.model.qs)):
                    self.model.qs[i] = np.zeros(len(self.model.qs[i]))
                self.model.qs[0][self.start_node_id] = 1.0  # Set belief at start_node_id for pose state

                self.get_logger().info(f'Model reset to start node {self.start_node_id} at pose {start_pose}')

            ob_id = None
            if self.skip_double_check_visited_state:
                ob_id = self.get_state_observation()
                obstacle_dist_per_actions = None
                ob_match_score = []
            #If we have no observation, get one.
            if ob_id is None:
                obstacle_dist_per_actions, ob_id, ob_match_score = self.get_panorama(len(self.model.get_possible_actions()))
            # qs = self.model.get_belief_over_states()
            # qo = self.get_expected_observation(qs)
            # self.last_ob_id = np.argmax(qo[0]) #might be imbricated list. to check
            self.last_ob_id = ob_id
            self.prev_scans_dist = obstacle_dist_per_actions

        self.set_navigation_mode(self.goal_id)
        self.is_goal_reaching_mode = (goal_ob_id >= 0) or (goal_pose_id >= 0)

        # IF WE WANT TO VISUALISE MCTS RESULTS ON RVIZ
        self.mcts_visualizer = MCTSRewardVisualiser(self, self.model, frame_id='map')

        next_possible_actions = self.model.define_next_possible_actions(obstacle_dist_per_actions, restrictive=True,logs=self.get_logger())
        ideal_next_action = [-1]
        data = None
        if self.model_imagine_next_action :
            ideal_next_action, data = self.model.define_actions_from_MCTS_run(num_steps=self.policy_length, observations=[self.last_ob_id],next_possible_actions=next_possible_actions, save_action_memory = False, logging= self.get_logger(),  plot_MCTS_tree=True)

            self.get_logger().info(f'THE IDEAL NEXT MOTION(S) (FOR MCTS):{ideal_next_action}')

            # Visualize MCTS rewards if we have the tree data
            if data is not None and 'plot_MCTS_tree' in data and data['plot_MCTS_tree'] is not None:
                self.mcts_visualizer.publish_mcts_rewards(data['plot_MCTS_tree'], top_k=5)
                self.get_logger().info('Published MCTS reward visualization to RViz')
        
        self.save_data_process(ob_id, ob_match_score,\
                      obstacle_dist_per_actions, elapsed_time=0.0,  data=data)
        
        reachable_points = []
        self.get_logger().info('AIF setup at path %s' % self.test_folder)
        next_possible_nodes = []
        for a in next_possible_actions:
            next_pose, next_pose_id = self.model.determine_next_pose(a)
            pt = Point()
            next_pose[0] = float(next_pose[0])
            next_pose[1] = float(next_pose[1])
            pt.x = float(next_pose[0])
            pt.y = float(next_pose[1])
            reachable_points.append(pt)
            self.publish_vector(next_pose[0], next_pose[1], self.actions_pub[a])
            self.get_logger().info('possible action %d, next node %d and pose %s' % (a, next_pose_id, str(next_pose)))
            next_possible_nodes.append(next_pose_id)
            if a == ideal_next_action[0]:
                #GOAL POSE
                self.pub_goal_pose(next_pose)

        # Publish possible actions, nodes, and goals for GUI
        self.publish_possible_actions_nodes(next_possible_actions, next_possible_nodes, reachable_points)

        # Save model at initialization if in goal-reaching mode (new test folder created)
        if self.is_goal_reaching_mode and test_id != 'None':
            self.save_model(self.test_folder)
            self.get_logger().info(f'Goal-reaching mode: Initial model saved to {self.test_folder}')

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

    def map_callback(self, msg):
        """Callback to receive and store the latest map from /map topic"""
        if self.latest_map is None:
            self.get_logger().info('Received first map from /map topic')
        self.latest_map = msg

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

    def publish_possible_actions_nodes(self, actions, nodes, reachable_points):
        """ Publish possible actions, nodes, and reachable goal coordinates for GUI """
        # Store data locally for navigation result matching
        self.current_possible_actions = actions.copy()
        self.current_possible_nodes = nodes.copy()
        self.current_reachable_goals = [(pt.x, pt.y) for pt in reachable_points]

        actions_msg = Int32MultiArray()
        actions_msg.data = actions
        self.possible_actions_pub.publish(actions_msg)

        nodes_msg = Int32MultiArray()
        nodes_msg.data = nodes
        self.possible_nodes_pub.publish(nodes_msg)

        # Flatten reachable_points (list of Point objects) to [x1, y1, x2, y2, ...]
        goals_msg = Float64MultiArray()
        goals_coords = []
        for pt in reachable_points:
            goals_coords.append(float(pt.x))
            goals_coords.append(float(pt.y))
        goals_msg.data = goals_coords
        self.reachable_goals_pub.publish(goals_msg)

        self.get_logger().info(f'Published {len(actions)} possible actions, {len(nodes)} nodes, and {len(reachable_points)} goals to GUI topics')

    def navigation_result_callback(self, msg):
        """
        Callback when navigation result is received from nav2_client.
        Finds the closest matching action and triggers AIFProcess action.
        """
        self.get_logger().info(f'Received navigation result: goal_reached={msg.goal_reached}, '
                              f'final_pose=[{msg.final_pose_x:.2f}, {msg.final_pose_y:.2f}], '
                              f'goal_pose=[{msg.goal_pose_x:.2f}, {msg.goal_pose_y:.2f}]')

        # Find the closest matching goal from the current reachable goals
        if len(self.current_reachable_goals) == 0:
            self.get_logger().warn('No reachable goals stored - cannot match navigation result')
            return

        goal_pose = (msg.goal_pose_x, msg.goal_pose_y)

        # Find the closest matching goal
        min_distance = float('inf')
        matched_index = -1

        for i, stored_goal in enumerate(self.current_reachable_goals):
            distance = np.sqrt((stored_goal[0] - goal_pose[0])**2 +
                             (stored_goal[1] - goal_pose[1])**2)
            if distance < min_distance:
                min_distance = distance
                matched_index = i

        if matched_index == -1:
            self.get_logger().error('Could not match navigation result to any stored goal')
            return

        matched_action = self.current_possible_actions[matched_index]
        matched_node = self.current_possible_nodes[matched_index]
        matched_goal = self.current_reachable_goals[matched_index]

        self.get_logger().info(f'Matched navigation result to action {matched_action}, '
                              f'node {matched_node}, goal {matched_goal} (distance: {min_distance:.3f}m)')

        # Trigger AIFProcess action with the matched action and goal_reached status
        self.send_aif_goal(matched_action, msg.goal_reached)

    def send_aif_goal(self, action, goal_reached):
        """
        Send goal to AIFProcess action server.
        """
        if not self.aif_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('AIFProcess action server not available!')
            return

        # action_msg = Int16()
        # action_msg.data = action
        goal_msg = AIFProcess.Goal()
        goal_msg.action = action
        goal_msg.goal_reached = goal_reached

        self.get_logger().info(f'Sending AIFProcess goal: action={action}, goal_reached={goal_reached}')

        send_goal_future = self.aif_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.aif_goal_response_callback)

    def aif_goal_response_callback(self, future):
        """
        Callback when AIFProcess goal is accepted/rejected.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('AIFProcess goal was rejected')
            return

        self.get_logger().info('AIFProcess goal accepted')

        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.aif_result_callback)

    def aif_result_callback(self, future):
        """
        Callback when AIFProcess action completes.
        """
        result = future.result().result
        self.get_logger().info(f'AIFProcess action completed with {len(result.possible_actions)} possible actions')

    def initialise_model(self, n_actions:int)-> None:
        """ With first panorama.
          We setup the model and incorporates the first ghost nodes
        """           
       
        #TODO: UPDATE PANORAMA TO KNOW ACTION RANGE
        obstacle_dist_per_actions, ob_id, ob_match_score = self.get_panorama(n_actions)
        #create model
        self.model = Ours_V5_RW(num_obs=2, num_states=2, dim=2, \
                                observations=[ob_id], \
                                n_actions=self.n_actions, influence_radius=self.influence_radius,\
                                robot_dim=self.robot_dim, lookahead_node_creation=self.lookahead_node_creation)
        
        self.model.set_memory_views(self.Views.get_memory_views())
        self.model.update_transition_nodes(obstacle_dist_per_actions=obstacle_dist_per_actions, logs=self.get_logger())
        self.model.update_C_dim()
        self.save_model(self.test_folder)
        
        

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
        if isinstance(goal_action, Int16) :
            goal_action = goal_action.data
        goal_success = goal_handle.request.goal_reached
        self.executed_action = Int16() #safety 
        self.executed_action.data = goal_action
        self.executed_action_pub.publish(self.executed_action)
        self.executed_action_pub.publish(self.executed_action) #safety
        self.executed_action_pub.publish(self.executed_action) #safety
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

            if self.model_imagine_next_action:
                ideal_next_action, data = self.model.define_actions_from_MCTS_run(num_steps=self.policy_length, observations=[self.last_ob_id],next_possible_actions=possible_actions, save_action_memory = False, logging= self.get_logger(), plot_MCTS_tree=True)
                self.get_logger().info(f'THE IDEAL NEXT MOTION(S) (FOR MCTS):{ideal_next_action}')
                next_pose, next_pose_id = self.model.determine_next_pose(ideal_next_action[0])
                self.pub_goal_pose(next_pose)

                # Visualize MCTS rewards after failed action
                if self.mcts_visualizer is not None and data is not None and 'plot_MCTS_tree' in data and data['plot_MCTS_tree'] is not None:
                    self.mcts_visualizer.publish_mcts_rewards(data['plot_MCTS_tree'], top_k=5)
                
            elapsed_time = time.time() - self.start_time
            self.save_model(self.test_folder)
            save_failed_step_data(copy.deepcopy(self.model), self.last_ob_id, np.array([0,0]), [0], possible_actions, self.prev_scans_dist, self.robot_pose, action_success=False, elapsed_time=elapsed_time, store_path=self.test_folder, action_select_data=data)

            
            
            result.possible_actions = possible_actions
            result.reachable_goals = [Point()]
            #GOAL POSE
            

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
        ideal_next_action = [-1]
        data = None
        if self.model_imagine_next_action:
            ideal_next_action, data = self.model.define_actions_from_MCTS_run(num_steps=self.policy_length, observations=[ob_id],next_possible_actions=next_possible_actions, save_action_memory = False, logging= self.get_logger(), plot_MCTS_tree=True)

            self.execution_time += time.time() - start_execution_time
            self.get_logger().info(f'THE IDEAL NEXT MOTION(S) (FOR MCTS):{ideal_next_action}')

            # Visualize MCTS rewards after each action
            if self.mcts_visualizer is not None and data is not None and 'plot_MCTS_tree' in data and data['plot_MCTS_tree'] is not None:
                self.mcts_visualizer.publish_mcts_rewards(data['plot_MCTS_tree'], top_k=5)
        elapsed_time = time.time() - self.start_time
        self.save_data_process(ob_id, ob_match_score, obstacle_dist_per_actions=obstacle_dist_per_actions, elapsed_time=elapsed_time, data= data)

        self.prev_scans_dist = obstacle_dist_per_actions
        self.last_ob_id = ob_id

        reachable_points = []
        next_possible_nodes = []
        for a in next_possible_actions:
            next_pose, next_pose_id = self.model.determine_next_pose(a)
            pt = Point()
            pt.x = next_pose[0]
            pt.y = next_pose[1]
            reachable_points.append(pt)
            self.publish_vector(next_pose[0], next_pose[1], self.actions_pub[a])
            self.get_logger().info('possible action %d next node %d and pose %s' % (a, next_pose_id, str(next_pose)))
            next_possible_nodes.append(next_pose_id)
            if a == ideal_next_action[0]:
                self.pub_goal_pose(next_pose)
                # self.get_logger().info(f'Publishing goal pose {next_pose} for action {a}')

        # Publish possible actions, nodes, and goals for GUI
        self.publish_possible_actions_nodes(next_possible_actions, next_possible_nodes, reachable_points)

        result.possible_actions = next_possible_actions
        result.possible_nodes = next_possible_nodes
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
                      obstacle_dist_per_actions:list,elapsed_time:float, data:dict=None):

        ob = self.Views.views[ob_id].full_ob
        if self.robot_pose is not None and len(self.robot_pose) >= 2 :
            robot_pose = self.robot_pose
        else:
            robot_pose = self.model.current_pose
    
        self.save_model(self.test_folder)
        save_step_data(self.model, ob_id, ob, ob_match_score, obstacle_dist_per_actions,\
                    robot_pose, action_success=True, elapsed_time=elapsed_time,\
                        store_path=self.test_folder, action_select_data=data, execution_time=self.execution_time,\
                        map_msg=self.latest_map)
        # if data is not None and 'poses_efe' in data:
        #     save_efe_plot(data['poses_efe'],self.model.get_current_timestep(),store_dir)

    def model_step_process(self, action: int, pose_id:int=None):
        agent_possible_directions = self.model.get_possible_actions()

        ob_id = None
        #If we "STAY" we check the state
        if self.skip_double_check_visited_state and ('STAY' in agent_possible_directions.keys() and action != agent_possible_directions['STAY']):
            ob_id = self.get_state_observation()
            obstacle_dist_per_actions = None
            ob_match_score = []
        #If we have no observation, get one.
        if ob_id is None:
            obstacle_dist_per_actions,ob_id, ob_match_score = self.get_panorama(len(agent_possible_directions))

        start_time = time.time()
        if 'STAY' in agent_possible_directions:
            self.model.next_possible_actions.append([agent_possible_directions['STAY'], 1])

        self.model.agent_step_update(action, [ob_id, pose_id, obstacle_dist_per_actions], logs=self.get_logger())
        self.execution_time += time.time() - start_time
        return obstacle_dist_per_actions, ob_id, ob_match_score
    
    def get_state_observation(self):
        next_pose_id = self.model.get_current_pose_id()
        ob_id = self.model.state_most_likely_observation(next_pose_id)
        
        self.get_logger().info(f'We have node {next_pose_id} with ob_id {ob_id}.')
        return ob_id
    
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
    
    def set_navigation_mode(self, goal_id)->None:
        """ Check if we have a goal and if the goal is valid.
        If no (valid) goal, we explore, else we desire to reach goal with a set weight on the preference
        """
        ob_id = None
        

        # Check if goal_path is a string (legacy mode with image matching)
        if isinstance(self.goal_path, str) and self.goal_path != 'None':
            img = process_path(self.goal_path)
            if img is not None:
                img = remove_white_border(img)
                try:
                    ob_id, ob_match_score = self.Views.get_closest_view_id(img,None)
                except Exception as e:
                    self.get_logger().warning(str(e))
                if not isinstance(ob_id,int):
                    self.get_logger().info(str(ob_id) + str(type(ob_id)) + 'is not a proper integer, matching scores: '+str(ob_match_score)+', goal set aborted')
                    ob_id = None
            if ob_id is not None:
                goal_id = [ob_id, -1]  # Use image-matched ob_id with no pose
        # Otherwise, check if goal IDs are specified directly (new mode)
        else:
            # Validate each goal_id element against model bounds
            has_valid_goal = False
            for i in range(len(goal_id)):
                if goal_id[i] >= 0:  # -1 means not specified
                    if goal_id[i] < len(self.model.A[i]):
                        self.get_logger().info(f'Goal ID[{i}]={goal_id[i]} is valid')
                        has_valid_goal = True
                    else:
                        self.get_logger().error(f'Goal ID[{i}]={goal_id[i]} is out of bounds (model.A[{i}] has {len(self.model.A[i])} elements). Setting to -1.')
                        goal_id[i] = -1
                        

            if not has_valid_goal:
                goal_id = None  # No valid goals specified

        if goal_id is not None and any(g >= 0 for g in goal_id):
            self.get_logger().info(f'We are aiming for goal {goal_id}')
            # We give the goal IDs as objective
            self.model.goal_oriented_navigation(goal_id, pref_weight = 10.0)
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

    # Parse command line arguments
    test_id = 'None'
    goal_ob_id = -1
    goal_pose_id = -1
    start_node_id = -1
    influence_radius = 1.6
    n_actions = 17
    lookahead_node_creation = 8
    skip_double_check = False

    # Parse arguments (format: -arg_name value)
    i = 1
    while i < len(sys.argv):
        if sys.argv[i] == '-test_id' and i + 1 < len(sys.argv):
            test_id = sys.argv[i + 1]
            i += 2
        elif sys.argv[i] == '-goal_ob_id' and i + 1 < len(sys.argv):
            goal_ob_id = int(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '-goal_pose_id' and i + 1 < len(sys.argv):
            goal_pose_id = int(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '-start_node_id' and i + 1 < len(sys.argv):
            start_node_id = int(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '-influence_radius' and i + 1 < len(sys.argv):
            influence_radius = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '-n_actions' and i + 1 < len(sys.argv):
            n_actions = int(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '-lookahead_node_creation' and i + 1 < len(sys.argv):
            lookahead_node_creation = int(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '-skip_double_check' and i + 1 < len(sys.argv):
            skip_double_check = sys.argv[i + 1].lower() == 'true'
            i += 2
        else:
            i += 1

    node = AIFProcessServer(test_id=test_id, goal_ob_id=goal_ob_id, goal_pose_id=goal_pose_id, start_node_id=start_node_id, influence_radius=influence_radius, n_actions=n_actions, lookahead_node_creation=lookahead_node_creation, skip_double_check_visited_state=skip_double_check)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
