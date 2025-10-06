#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from aimapp_actions.action import Panorama    
from action_msgs.msg import GoalStatus
import argparse
import time

class Panorama360CamClient(Node):

    def __init__(self):
        super().__init__('panorama_client')
        self.get_logger().info('panorama client node has been started.')
        self.get_panorama = ActionClient(self,Panorama, 'get_360_image')     
        self.panorama_status = GoalStatus.STATUS_EXECUTING
        self.panorama_result = None                             

    def turn_to_get_panorama(self, n_turn_stops:int=2,n_actions:int=12):
        '''
        turn 360degree and take n_turn_stops image of the surrounding. 
        n_turn_stops must be >=0 to the number of direction the agent 
        can take, the agent is advised to take images in the directions it can go
        return result
        '''
        panorama_future = self.send_panorama_goal(n_turn_stops, n_actions)
        rclpy.spin_until_future_complete(self, panorama_future)

        timeout = 140  # seconds
        start_time = time.time()
        
        while (self.panorama_status not in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED]) and \
            (time.time() - start_time < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)

        # self.get_logger().info('panorama client returning results.'+str(self.panorama_status))
        return self.panorama_result

    def send_panorama_goal(self, n_turn_stops:int=2,n_actions:int=12):
        """ 
        ACTION
        Receive the number of stop to do during a 360* turn 
        and send it to the service to get panorama 
        """
        goal_msg = Panorama.Goal()
        goal_msg.goal_angles = [0.0]
        goal_msg.n_actions = n_actions

        self.get_panorama.wait_for_server()
        self.panorama_status = GoalStatus.STATUS_EXECUTING

        future = self.get_panorama.send_goal_async(goal_msg, feedback_callback=self.pano_feedback_callback)
        future.add_done_callback(self.pano_goal_response_callback) 

        return future
    
    def pano_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Panorama goal was rejected.")
            self.panorama_status = GoalStatus.STATUS_REJECTED
            return

        self.get_logger().info("Panorama goal accepted.")
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.pano_result_callback)


    def pano_result_callback(self, future):
        try:
            result = future.result()
            self.panorama_result = result.result
            self.panorama_status = result.status
            # self.get_logger().info(f"Panorama action completed with status: {result.status}")
        except Exception as e:
            self.get_logger().error(f"Panorama result callback failed: {e}")
            self.panorama_status = GoalStatus.STATUS_ABORTED

    def pano_feedback_callback(self, feedback_msg):
        """ Get the panorama action feedback"""
        feedback = feedback_msg.feedback
        #self.get_logger().info('Panorama feedback current goal angle: {0}'.format(feedback.current_stop))
       
    
def main(x):
    rclpy.init()

    panorama_client = Panorama360CamClient()
    n_turn = x
    future = panorama_client.turn_to_get_panorama(n_turn)
    
    # rclpy.spin_until_future_complete(panorama_client, future)
    rclpy.spin(panorama_client)
    panorama_client.get_logger().info('Done')
    # %s' %  str(response.status))

    panorama_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='how many stops')
    parser.add_argument('--x', type=int, default=2,  help='how many stops')
    args = parser.parse_args()
    main(args.x)