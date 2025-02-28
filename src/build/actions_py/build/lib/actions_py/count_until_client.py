#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from my_robot_interfaces.action import CountUntil


class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__('count_until_client')
        self.count_until_client = ActionClient(
            self, 
            CountUntil, 
            "count_until")
        
    def send_goal(self, target_number, period):
        # Wait for the action server
        self.count_until_client.wait_for_server()
        
        # Create a goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period
        
        # Send the goal
        self.get_logger().info('Sending Goal....')
        self.count_until_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle: ClientGoalHandle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Goal accepted by the server.')
            goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info('Goal rejected by the server.')

    def goal_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f'Result: {result.reached_number}')
        except Exception as e:
            self.get_logger().error(f'Error while getting the result: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(6, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
