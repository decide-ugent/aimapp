#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import threading
from map_dm_nav_actions.action import PotentialField
import numpy as np
import tf2_ros 
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
class PotentialFieldAction(Node):
    def __init__(self, cmd_linear_max:float= 0.2, cmd_angular_max:float=0.2, 
                 angular_goal_tolerance:float = np.pi/10,
                 distance_goal_tolerance:float = 0.1, obstacle_tolerance:float = 0.8):
        super(PotentialFieldAction, self).__init__("PotentialFieldAction")
        
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
            '/odometry/filtered',
            self.odom_callback,
            qos_profile=qos_policy
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            'scan_filtered',
            self.lidar_callback,
            qos_profile=qos_policy
        )

        self.apf_attraction = self.create_publisher(
            msg_type=PoseStamped,
            topic="APF_attraction",
            qos_profile=qos_policy)
        
        self.apf_repulsion = self.create_publisher(
            msg_type=PoseStamped,
            topic="APF_repulsion",
            qos_profile=qos_policy)
        
        self.apf_final = self.create_publisher(
            msg_type=PoseStamped,
            topic="APF_final",
            qos_profile=qos_policy)
        
        self.tf_theta = None
        self.robot_pos_xy = None
        self.last_timer_position = None
        self.robot_theta = np.array([])
        
        self.V_repulsion = None
        self.obstacle_tolerance = obstacle_tolerance

        self.cmd_angular_max = cmd_angular_max
        self.cmd_linear_max = cmd_linear_max
        self.linear_vel_reductor = 0.5
        self.angular_vel_reductor = 0.3

        self.angular_goal_tolerance = angular_goal_tolerance
        self.distance_goal_tolerance = distance_goal_tolerance
        self.delta_distance = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.execution_rate =  self.create_rate(5) #sec = 1/Hz
        
        self.timer = None
        #test_potential_field = self.create_timer(timer_period_sec=1, callback=self.execute_potential_field)
        #self.service_potential_field = self.create_service(PotentialField, 'potential_field', self.execute_potential_field)    
        
        

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.action_server = ActionServer(
            node=self,
            action_type=PotentialField,
            action_name="potential_field",
            execute_callback=self.execute_potential_field,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            result_timeout=2000)
        
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


    def goal_callback(self, goal_handle):
        self.get_logger().info('potential field received goal request')
        return GoalResponse.ACCEPT

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
    
    def odom_callback(self, odom:Odometry):
        # print('IN ODODM', odom)
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

    def lidar_callback(self, lidar_msg:LaserScan):
        """ get repulsion field from scan"""

        # Define the Field of View (e.g., 180° or 90° in radians)
        fov_half = np.deg2rad(30)  

        angle_min = lidar_msg.angle_min
        angle_max = lidar_msg.angle_max
        range_max = lidar_msg.range_max
        range_min = lidar_msg.range_min
        step = lidar_msg.angle_increment
     
        scan = np.array(lidar_msg.ranges)
        scan_range  = len(scan)

        x_r = 0.0000001
        y_r = 0.0000001

        if self.tf_theta is None:
            theta_offset = 0.0
        else:
            theta_offset = self.tf_theta

        self.V_repulsion = [x_r,y_r]
        if isinstance(self.robot_theta,np.ndarray) and len(self.robot_theta)== 0:
            return
        self.get_logger().info(' ')
        for i in range(scan_range): 
            angle = angle_min + step * i + self.robot_theta + theta_offset # Calculate the absolute angle of the scan point

            angle_difference = normalise_angle(angle - self.robot_theta) 
            # Consider only obstacles within the FOV in front of the robot
            if abs(angle_difference) <= fov_half:
                #If the value of the scan is > 3.5m it's above the lidar scan range
                if(np.isfinite(scan[i]) and scan[i] <= range_max and scan[i] >= range_min):
                    Q_repulsion = 1
                    Current_Q = Q_repulsion / (4 * np.pi * pow(scan[i],2))
                    #Current_Q = Q_repulsion / pow(scan[i],2) * self.compute_force(angle, angle_max)
                    #Projection of the vectors in the x , y coordinates
                    x_r -= Current_Q * np.cos(angle_difference) * self.obstacle_tolerance
                    y_r -= Current_Q * np.sin(angle_difference) * self.obstacle_tolerance
                    self.get_logger().info("reulsion at absolute: %f, relative %f, dist %f, rep x %f, y %f" % (np.rad2deg(angle), np.rad2deg(angle_difference), round(scan[i], 2), round(Current_Q * np.cos(angle_difference)* self.obstacle_tolerance,4), round(Current_Q * np.sin(angle_difference)* self.obstacle_tolerance,4)))
        self.V_repulsion = [x_r, y_r]

        # angle_of_rep = np.arctan(self.V_repulsion[1]/self.V_repulsion[0])*180/np.pi
        # print(angle_of_rep)
        
        # self.get_logger().info("the angle of repulsion is : %f" % angle_of_rep)
     
    def execute_potential_field(self, goal_handle): #, goal_pose:Point): #Goal is a point
        
        goal_pose = goal_handle.request.goal_pose
        feedback_msg = PotentialField.Feedback()
        result = PotentialField.Result()

        #THIS WILL BE THE ACTION CALLBACK
        self.get_logger().info("Received a goal from client")
        self.start_motion_timer()
        self.execution_rate.sleep()
        goal_reached = False

        if self.tf_theta is None:
            self.get_tf_transform()
        
        while not goal_reached and rclpy.ok():
            # self.get_logger().info(str(goal_pose))
            
            #self.get_logger().info('GOAL:'+str(goal_pose))
            #self.get_logger().info('POSE:'+str(self.robot_pos_xy)+str(self.robot_theta))
            #print('self.V_repulsion',self.V_repulsion)
            if self.action_goal_exceptions(goal_handle):
                result.goal_reached = goal_reached
                result.pose = [np.round(self.robot_pos_xy[0],3), np.round(self.robot_pos_xy[1],3), np.round(self.robot_theta,4)]
                return result
            if self.robot_pos_xy is None or self.V_repulsion is None:
                self.execution_rate.sleep()
                continue

            #print('reflecting')
            
            V_attraction = self.compute_attraction(np.array([goal_pose.x,goal_pose.y]))
            
            self.publish_vector(self.V_repulsion[0], self.V_repulsion[1], self.apf_repulsion)
            #angle_of_rep = np.arctan(self.V_repulsion[1]/self.V_repulsion[0])*180/np.pi
            # self.get_logger().info("the angle of repulsion is : %f" % angle_of_rep)
            # self.get_logger().info("the repulsion is : %s" % str(self.V_repulsion))
            # self.get_logger().info("the attraction is : %s" % str(V_attraction))

            x_final = V_attraction[0] + self.V_repulsion[0]
            y_final = V_attraction[1] + self.V_repulsion[1]
            
            self.publish_vector(x_final, y_final, self.apf_final)
            angle_target = np.arctan2(y_final, x_final)
            angle_delta = angle_target - self.robot_theta
            # print('angle_delta before normalisation:', angle_delta)
            angle_delta = normalise_angle(angle_delta)

            #self.get_logger().info("angle to goal: %f, distance to goal: %f" % (angle_delta, self.delta_distance))
            feedback_msg.dist_to_goal = self.delta_distance
            goal_handle.publish_feedback(feedback_msg)
            
            v_in, w_in = self.compute_velocities(self.delta_distance, angle_delta)
            # print('linear vel, angular vel:', v_in, w_in)
            direction = Twist()
            if(abs(angle_delta) > self.angular_goal_tolerance) \
                and self.delta_distance > self.distance_goal_tolerance \
                and abs(w_in) >= 0.05: #Don't want to wait forever cause of Rep
                # #turn
                direction.angular.z = w_in
                direction.linear.x  = 0.0
            elif self.delta_distance > self.distance_goal_tolerance :
                #forward
                direction.angular.z = 0.0
                direction.linear.x  = v_in
            elif abs(w_in) < 0.05:
                direction.angular.z = 0.0
                direction.linear.x  = 0.0
                goal_reached = True
            else:
                # print('LAST ELSE')
                direction.angular.z = 0.0
                direction.linear.x  = 0.0
                goal_reached = True

            # print('vel:', direction)
            self.cmd_pub.publish(direction)
            self.execution_rate.sleep()
            #time.sleep(self.execution_rate)
            #print('end sleep')
        
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
    
    def compute_attraction(self,goal_pose:np.ndarray)->list:
        #Compute distance between the attraction and the current position
        self.delta_distance = euclidian_distance(goal_pose, self.robot_pos_xy)

        #Compute the point to reach relative to the current position
        x_a = goal_pose[0] - self.robot_pos_xy[0]
        y_a = goal_pose[1] - self.robot_pos_xy[1]

        Q_attraction = 100
        #Create the Module of the force to simulate
        F_attraction = (Q_attraction )/(4 * np.pi * pow(self.delta_distance,2))
        #Create the position of the force to simulate
        V_attraction = [F_attraction * x_a , F_attraction * y_a]

        #self.get_logger().info("diff goal/robot x : %f | y : %f" % (x_a,y_a))
        #self.get_logger().info("Force: %f" % F_attraction)

        # angle_attract = normalise_angle(np.arctan2(V_attraction[1],V_attraction[0]))
        #self.get_logger().info("angle attraction :%f°" % angle_attract)
        #self.get_logger().info("v_attraction is : x = %f ; y = %f" % (V_attraction[0],V_attraction[1]))

        self.publish_vector(V_attraction[0], V_attraction[1], self.apf_attraction)

        return V_attraction

        # geometry_msgs::msg::PoseStamped attraction = PublishVector(V_attraction[0],V_attraction[1]);
        # att_pub->publish(attraction);

    def compute_velocities(self,delta_distance, angle_delta):
        #Reduce velocity when we get closer to goal
        v_in = self.linear_vel_reductor * delta_distance
        w_in = self.angular_vel_reductor * angle_delta

        #CLIP
        v_in = max(min(v_in, self.cmd_linear_max), -self.cmd_linear_max)
        w_in = max(min(w_in, self.cmd_angular_max), -self.cmd_angular_max)

        # w_in = np.sign(w_in)*max(0.02,abs(w_in)) #I do not wish to wait forever to turn

        return float(v_in), float(w_in)
    
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
    
    def publish_vector(self,x,y, name_pub):
        """" Visualise vectors in rviz with their intensity (topped at 7m from robot position, just for visualisation purposes)"""
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

    # def start_motion_timer(self):
    #     if self.timer is not None:
    #         self.timer.reset()
    #     else:
    #         self.create_timer(10, self.check_movement)
    
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

    def get_tf_transform(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'laser',      # source frame
                rclpy.time.Time())  # get the latest available
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("TF from base_link to laser not available.")
            return 
        
        euler_angles = ros_quaternion_to_euler(tf.transform.rotation)
        theta = np.round(euler_angles[2], 4)
        #I want theta between 0-4*pi, not 0-2pi/-2pi-0
        if theta < 0:
            theta += 2*np.pi
        
        self.tf_theta = theta
        self.get_logger().info(f'tf theta: {theta}')
           

def normalise_angle(angle):
    """Normalise an angle to be within the range -pi to pi."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

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

def main():
    rclpy.init()
    # print('HERE')
    potential_field = PotentialFieldAction()
    
    # rclpy.spin(potential_field)

    multi_thread_executor = MultiThreadedExecutor(num_threads=4)
    multi_thread_executor.add_node(potential_field)
    rclpy.spin(potential_field, executor=multi_thread_executor)


    print('DESTROY')
    potential_field.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
