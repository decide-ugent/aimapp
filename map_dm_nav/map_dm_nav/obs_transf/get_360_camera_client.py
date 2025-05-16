#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from map_dm_nav_actions.action import Panorama    
from action_msgs.msg import GoalStatus
import argparse


class Panorama360CamClient(Node):

    def __init__(self):
        super().__init__('panorama_client')
        self.get_logger().info('panorama client node has been started.')
        self.get_panorama = ActionClient(self,Panorama, 'get_360_image')     
        self.panorama_status = GoalStatus.STATUS_EXECUTING
        self.panorama_result = None                             

    def turn_to_get_panorama(self, n_turn_stops:int=2, n_actions:int=12):
        '''
        turn 360degree and take n_turn_stops image of the surrounding. 
        n_turn_stops must be >=0 to the number of direction the agent 
        can take, the agent is advised to take images in the directions it can go
        return result
        '''
        panorama_future = self.send_panorama_goal(n_turn_stops, n_actions)
        rclpy.spin_until_future_complete(self, panorama_future)
        # print(panorama_future.__dict__)
        # print(highlevelnav.panorama_status)
        
        while self.panorama_status != GoalStatus.STATUS_SUCCEEDED:
            rclpy.spin_once(self)

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
        future = self.get_panorama.send_goal_async(goal_msg,feedback_callback=self.pano_feedback_callback)
        
        future.add_done_callback(self.pano_goal_response_callback)
        return future
    
    def pano_goal_response_callback(self, future):
        """ Get regular goal_responses"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_pano_callback)

    def get_result_pano_callback(self, future):
        """ Get pano action result"""
        result = future.result().result
        # self.get_logger().info('Result: {0}'.format(result.sequence))
        self.panorama_status = GoalStatus.STATUS_SUCCEEDED
        self.panorama_result = result

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