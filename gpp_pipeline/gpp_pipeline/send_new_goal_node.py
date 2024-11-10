#!/usr/bin/env python3
import os
import yaml

from time import sleep
from typing import Dict

import rclpy
from rclpy.action import ActionClient
import rclpy.clock
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import TransitionEvent
from tf_transformations import quaternion_from_euler
from rclpy.executors import ExternalShutdownException
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class SendNewGoalNode(Node):
    def __init__(self) -> None:
        super().__init__('send_new_goal')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.declare_parameter('target_robot_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('target_robot_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('target_robot_phi', rclpy.Parameter.Type.DOUBLE)

        self._target_robot_x: float = self.get_parameter("target_robot_x").value
        self._target_robot_y: float = self.get_parameter("target_robot_y").value
        self._target_robot_phi: float = self.get_parameter("target_robot_phi").value

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info("send_goal_timeout: Canceled")
        self._send_goal_timeout_timer.cancel()
        self._path_planning_timeout_timer.reset()
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(str(result)))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        self.get_logger().info("shutdown now...")
        # Shutdown after receiving a result
        rclpy.shutdown()
        self.get_logger().info("shutdown finished...")

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        # goal_pose.header.stamp  = rclpy.time.Time().to_msg()
        goal_msg.pose.header.stamp  = rclpy.clock.Clock().now().to_msg()
        goal_msg.pose.pose.position.x = self._target_robot_x
        goal_msg.pose.pose.position.y = self._target_robot_y
        goal_msg.pose.pose.position.z = 0.0
        tf_quaternion = quaternion_from_euler(0.0, 0.0, self._target_robot_phi)
        goal_msg.pose.pose.orientation.x = tf_quaternion[0]
        goal_msg.pose.pose.orientation.y = tf_quaternion[1]
        goal_msg.pose.pose.orientation.z = tf_quaternion[2]
        goal_msg.pose.pose.orientation.w = tf_quaternion[3]

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_timeout_timer = self.create_timer(5.0, self.send_goal_timeout_cb, clock=rclpy.clock.Clock())
        self._path_planning_timeout_timer = self.create_timer(20.0, self.path_planning_timeout_cb, clock=rclpy.clock.Clock())

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_goal_timeout_cb(self) -> None:
        # Retry sending goal as the bt_navigator has timed out
        self.get_logger().info("Resending goal")
        self.send_goal()

    def path_planning_timeout_cb(self) -> None:
        self.get_logger().info("Path Planning timeout detected")
        rclpy.shutdown()
        
def main(args = None):
    try:
        rclpy.init(args=args)
        action_client = SendNewGoalNode()
        action_client.send_goal()
        rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == "__main__":
    main()