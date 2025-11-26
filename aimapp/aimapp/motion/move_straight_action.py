#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import threading
from aimapp_actions.action import MoveStraight
import numpy as np
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
class MoveStraightAction(Node):
    def __init__(self, cmd_linear_max:float= 0.15, cmd_angular_max:float=0.2, 
                 angular_goal_tolerance:float = np.pi/10,
                 distance_goal_tolerance:float = 0.18):
        super(MoveStraightAction, self).__init__("MoveStraightAction")
        
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        
        self.cmd_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=qos_policy)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            qos_profile=qos_policy
        )

        self.robot_pos_xy = None
        self.robot_theta = np.array([])
        self.V_repulsion = None

        self.last_timer_position= None
        self.timer = None

        self.cmd_angular_max = cmd_angular_max
        self.cmd_linear_max = cmd_linear_max
        self.linear_vel_reductor = 1
        self.angular_vel_reductor = 0.5

        self.angular_goal_tolerance = angular_goal_tolerance
        self.distance_goal_tolerance = distance_goal_tolerance


        self.execution_rate =  self.create_rate(5) #sec = 1/Hz

        #test_potential_field = self.create_timer(timer_period_sec=1, callback=self.execute_motion)
        #self.service_potential_field = self.create_service(MoveStraight, 'potential_field', self.execute_motion)    
        
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.action_server = ActionServer(
            node=self,
            action_type=MoveStraight,
            action_name="move_straight",
            execute_callback=self.execute_motion,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            result_timeout=2000)
        

    def goal_callback(self, goal_handle):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def odom_callback(self, odom:Odometry):
 
        robot_xy, theta = self.extract_robot_xy_theta(
            odom_msg=odom
        )

        self.robot_pos_xy = robot_xy
        self.robot_theta = theta

    def extract_robot_xy_theta(self, odom_msg:Odometry):
        """Return array with x, y and also yaw in the tuple"""
        robot_x = np.round(
            odom_msg.pose.pose.position.x,
            4
        )
        robot_y = np.round(
            odom_msg.pose.pose.position.y,
            4
        )
        robot_pos_xy = np.array(
            [robot_x, robot_y]
        )

        quaternion = odom_msg.pose.pose.orientation
        euler_angles = ros_quaternion_to_euler(quaternion)

        theta = np.round(euler_angles[2], 4)

        return robot_pos_xy, theta

    def check_movement(self):
        if self.robot_pos_xy is not None:
            robot_pose = self.robot_pos_xy + [self.robot_theta]
            if self.last_timer_position is not None \
            and np.allclose(robot_pose, self.last_timer_position, atol=0.1):
                self.get_logger().info("Robot has not moved in the last 8 seconds. Starting abortion")
                self.last_timer_position = None
                self.stop_motion_timer()
                self.abort_goal_handle()
                return
            self.last_timer_position = robot_pose

    def abort_goal_handle(self):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()

    def handle_accepted_callback(self, goal_handle):
        self.abort_goal_handle()
        with self._goal_lock:
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def create_motion_timer(self):
        if self.timer is None:
            self.timer = self.create_timer(8.0, self.check_movement)
            #self.timer = threading.Timer(7.0, self.check_movement)
    
    def start_motion_timer(self):
        if self.timer is not None:
            # self.timer.start()
            self.timer.reset()
        else:
            self.timer = self.create_timer(8.0, self.check_movement)

    def stop_motion_timer(self):
        if self.timer is not None:
            self.timer.cancel()
        # angle_of_rep = np.arctan(self.V_repulsion[1]/self.V_repulsion[0])*180/np.pi
        # print(angle_of_rep)
        # self.get_logger().info("the angle of repulsion is : %f" % angle_of_rep)
     
    def execute_motion(self, goal_handle): #, goal_pose:Point): #Goal is a point
    
        goal_pose = goal_handle.request.goal_pose
        feedback_msg = MoveStraight.Feedback()
        result = MoveStraight.Result()
        self.start_motion_timer()

        #THIS WILL BE THE ACTION CALLBACK
        self.get_logger().info("Received a goal from client")
        self.execution_rate.sleep()
        goal_reached = False

        full_goal_pose = np.array([goal_pose.x,goal_pose.y, goal_pose.z])
        while not goal_reached and rclpy.ok():
            # self.get_logger().info(str(goal_pose))
            
            self.get_logger().info('GOAL:'+str(full_goal_pose))
            self.get_logger().info('POSE:'+str(self.robot_pos_xy) + str(self.robot_theta))
            if self.action_goal_exceptions(goal_handle):
                return MoveStraight.Result()
            while self.robot_pos_xy is None:
                self.execution_rate.sleep()
            goal_pose = full_goal_pose[:2]
            
            direction_vector = goal_pose - self.robot_pos_xy
            delta_distance = euclidian_distance(goal_pose, self.robot_pos_xy)
            #delta_distance = np.linalg.norm(direction_vector)
            
            angle_target = np.arctan2(direction_vector[1], direction_vector[0])
            angle_delta = angle_target - self.robot_theta
            # angle_delta = np.arctan2(self.robot_pos_xy[1] - goal_pose[1], self.robot_pos_xy[0] - goal_pose[0] )
            # if angle_delta < 0 :
            #     angle_delta += 2*np.pi

            print('angle and delta_distance',angle_delta, delta_distance)
            self.get_logger().info("angle to goal: %f, distance to goal: %f" % (angle_delta, delta_distance))
            feedback_msg.dist_to_goal = delta_distance
            goal_handle.publish_feedback(feedback_msg)

            angle_delta = normalise_angle(angle_delta)
            print('normalised angle', angle_delta)
            
            v_in, w_in = self.compute_velocities(delta_distance, angle_delta)
            print('linear vel, angular vel:', v_in, w_in)
            direction = Twist()
            if(abs(angle_delta) > self.angular_goal_tolerance) \
                and delta_distance > self.distance_goal_tolerance :
                # #turn
                direction.angular.z = w_in
                direction.linear.x  = 0.0
            elif delta_distance > self.distance_goal_tolerance :
                #forward
                direction.angular.z = 0.0
                direction.linear.x  = v_in
            else:
                print('LAST ELSE')
                direction.angular.z = 0.0
                direction.linear.x  = 0.0
                goal_reached = True

            print('vel:', direction)
            self.cmd_pub.publish(direction)
            self.execution_rate.sleep()
            #time.sleep(self.execution_rate)
            print('end sleep')
        
        direction = Twist()
        direction.angular.z = 0.0
        direction.linear.x  = 0.0
        self.cmd_pub.publish(direction)
        
        self.get_logger().info('goal_reached: %s'% str(goal_reached))
        

        pose = [np.round(self.robot_pos_xy[0],3), np.round(self.robot_pos_xy[1],3), np.round(self.robot_theta,4)]
        result.pose = pose
        result.goal_reached = goal_reached
        self.stop_motion_timer()
        return result
    
    def angle_between(a, b):
        angle = np.atan2(a[1] - b[1], b[0] - a[0])
        if angle < 0:
            angle += 2*np.pi
        return angle


    def compute_velocities(self, delta_distance, angle_delta):
        #Reduce velocity when we get closer to goal
        v_in = self.linear_vel_reductor * delta_distance
        w_in = self.angular_vel_reductor * angle_delta

        #CLIP
        v_in = max(min(v_in, self.cmd_linear_max), -self.cmd_linear_max)
        w_in = max(min(w_in, self.cmd_angular_max), -self.cmd_angular_max)

        # w_in = np.sign(w_in)*max(0.02,abs(w_in)) #I do not wish to wait forever to turn

        return v_in, w_in
    
    def action_goal_exceptions(self, goal_handle):
        if not goal_handle.is_active:
            self.get_logger().info('Goal aborted')
            direction = Twist()
            direction.linear.x = 0.0
            direction.angular.z = 0.0
            self.cmd_pub.publish(direction)
            return True

        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            direction = Twist()
            direction.linear.x = 0.0
            direction.angular.z = 0.0
            self.cmd_pub.publish(direction)
            return True
        return False

def ros_quaternion_to_euler(quaternion)-> np.ndarray:
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])

def min_delta(d1:list, d2:list, max_:float) -> float:
    delta = np.min([np.abs(d1 - d2), max_ - np.abs(d1 - d2)])
    return delta

#This is from odometry.py
def euclidian_distance(d1:list, d2:list)->int:
    sum = 0
    for i in range(len(d1)):
        sum += min_delta(d1[i],d2[i], np.inf)**2
    delta_dist = np.sqrt(sum)
    return delta_dist

def two_pi_complement(angle):
    """ return the inverse angle in a 360 circle,
    thus we can compare 360 with 0 
    """
    if (angle > 2*np.pi or angle < -2.0*np.pi):
      angle = angle%(2.0*np.pi)
    if(angle < 0):
      return (2*np.pi+angle)
    elif (angle > 0):
      return (-2*np.pi+angle)

    return(2*np.pi)

def normalise_angle(angle):
    """Normalise an angle to be within the range -pi to pi."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

def main():
    rclpy.init()
    print('HERE')
    potential_field = MoveStraightAction()
    # rclpy.spin(potential_field)

    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(potential_field)
    rclpy.spin(potential_field, executor=multi_thread_executor)


    print('DESTROY')
    potential_field.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
