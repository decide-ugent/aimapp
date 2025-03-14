#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from high_level_nav_actions.action import MoveStraight    
from action_msgs.msg import GoalStatus

class MSClient(Node):

    def __init__(self):
        super().__init__('move_straight_client')
        self.get_logger().info('move_straight_client node has been started.')
        # self.cli = self.create_client(MoveStraight, 'potential_field')    
        self.cli = ActionClient(self,MoveStraight, 'move_straight')     
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = MoveStraight.Request()    
        # 
        self.motion_status = GoalStatus.STATUS_EXECUTING
        self.motion_result = None                            

    def goal_response_callback(self, future):
        """ Get regular goal_responses"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ Get action result"""
        result = future.result().result
        # self.get_logger().info('Result: {0}'.format(result.sequence))
        self.motion_status = GoalStatus.STATUS_SUCCEEDED
        self.motion_result = result

    def pano_feedback_callback(self, feedback_msg):
        """ Get the motion action feedback"""
        feedback = feedback_msg.feedback
        # self.get_logger().info('motion feedback current dist to goal: {0}'.format(feedback.dist_to_goal))

    def send_goal(self, pose):
        goal_msg = MoveStraight.Goal()
        goal_msg.goal_pose = pose
        self.cli.wait_for_server()
        self.motion_status = GoalStatus.STATUS_EXECUTING
        future = self.cli.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        return future
    
    def go_to_pose(self, goal_pose:list)-> bool:
        """ convert pose to Point, send it to action and wait for result (goal reached)"""
        pose = Point(x=goal_pose[0], y=goal_pose[1])
        future = self.send_goal(pose)
        rclpy.spin_until_future_complete(self, future)

        while self.motion_status != GoalStatus.STATUS_SUCCEEDED:
            rclpy.spin_once(self)

        self.get_logger().info(
                'Goal'+ str(goal_pose) +'reached with potential field')
        
        return self.motion_result
        
def main(x,y):
    rclpy.init()

    move_straight_client = MSClient()

    pose = Point(x=x, y=y)

    future = move_straight_client.send_goal(pose)
    
    rclpy.spin_until_future_complete(move_straight_client, future)
    move_straight_client.get_logger().info(
                'Goal reached with potential field')
    # %s' %  str(response.status))

    move_straight_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Reach position x,y')
    parser.add_argument('--x', type=float, default=0.0,  help='x goal value')
    parser.add_argument('--y', type=float, default=0.0,  help='y goal value')
    args = parser.parse_args()
    main(args.x,args.y)