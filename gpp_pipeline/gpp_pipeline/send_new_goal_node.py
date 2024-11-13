#!/usr/bin/env python3
import os
import subprocess
import yaml

import rclpy
from rclpy.node import Node
import rclpy.clock
from rclpy.executors import ExternalShutdownException

from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from tf_transformations import quaternion_from_euler




class SendNewGoalNode(Node):
    def __init__(self) -> None:
        super().__init__('send_new_goal')
        self._navigate_to_pose_ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.declare_parameter('target_robot_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('target_robot_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('target_robot_phi', rclpy.Parameter.Type.DOUBLE)

        self.declare_parameter('resend_goal_timeout', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path_planning_timeout', rclpy.Parameter.Type.DOUBLE)

        self.declare_parameter('rejected_goal_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('planning_time_path', rclpy.Parameter.Type.STRING)

        self.declare_parameter('planning_attempt_index', rclpy.Parameter.Type.INTEGER)

        self._target_robot_x: float = self.get_parameter("target_robot_x").value
        self._target_robot_y: float = self.get_parameter("target_robot_y").value
        self._target_robot_phi: float = self.get_parameter("target_robot_phi").value
        self._resend_goal_timeout: float = self.get_parameter("resend_goal_timeout").value
        self._path_planning_timeout: float = self.get_parameter("path_planning_timeout").value
        
        self._rejected_goal_path: float = self.get_parameter("rejected_goal_path").value
        self._planning_time_path: float = self.get_parameter("planning_time_path").value

        self._planning_attempt_index: int = self.get_parameter("planning_attempt_index").value

        self._got_some_response: bool = False

        # with open("/home/rosjaeger/Desktop/rejected_goal.yaml", 'r') as file:
        #     self.data = yaml.safe_load(file)
        #     rejected_goal: bool = self.data["goal_rejected"]
        #     self.get_logger().info("rejected goal: " + str(rejected_goal))

        # with open("/home/rosjaeger/Desktop/rejected_goal.yaml", 'w') as file:
        #     data = yaml.safe_load(file)
        #     data["goal_rejected"] = False
        #     yaml.dump(self.data, file)

        # with open("/home/rosjaeger/Desktop/rejected_goal.yaml", 'r') as file:
        #     self.data = yaml.safe_load(file)
        #     rejected_goal: bool = self.data["goal_rejected"]
        #     self.get_logger().info("rejected goal: " + str(rejected_goal))

    def goal_response_callback(self, future):
        self._got_some_response = True
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            with open(self._rejected_goal_path, 'w') as file:
                data = dict()
                data["goal_rejected"] = True
                yaml.dump(data, file)
            return

        self.get_logger().info('Goal accepted :)')
        self._start_time = rclpy.clock.Clock().now().nanoseconds
        print(self._start_time)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        # self.get_logger().info("send_goal_timeout: Canceled")
        self._send_goal_timeout_timer.cancel()
        self._got_some_response = True
        # self._path_planning_timeout_timer.reset()
        
    def get_result_callback(self, future):
        self._got_some_response = True
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(str(result)))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        self._end_time = rclpy.clock.Clock().now().nanoseconds
        print(self._end_time)
        diff = self._end_time - self._start_time
        print("diff" + str(diff))

        data = None
        if os.path.isfile(self._planning_time_path):
            with open(self._planning_time_path, 'r') as file_r:
                data = yaml.safe_load(file_r)
        else:
            data = dict()

        with open(self._planning_time_path, 'w') as file_w:
            data[str(self._planning_attempt_index)] = diff
            yaml.dump(data, file_w)

        self.get_logger().info("shutdown now...")
        # Shutdown after receiving a result
        rclpy.shutdown()
        self.get_logger().info("shutdown finished...")

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._navigate_to_pose_ac.wait_for_server(timeout_sec=10)

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

        self._send_goal_future = self._navigate_to_pose_ac.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self._send_goal_timeout_timer = self.create_timer(self._resend_goal_timeout, 
                                                          self.send_goal_timeout_cb, 
                                                          clock=rclpy.clock.Clock())
        self._path_planning_timeout_timer = self.create_timer(self._path_planning_timeout, 
                                                              self.path_planning_timeout_cb, 
                                                              clock=rclpy.clock.Clock())

    def send_goal_timeout_cb(self) -> None:
        # Retry sending goal as the bt_navigator has timed out
        self.get_logger().info("Resending goal")
        self.send_goal()

    def path_planning_timeout_cb(self) -> None:
        self.get_logger().info("Path Planning timeout detected")
        # rclpy.shutdown()
        # self.destroy_node()

        if not self._got_some_response:
            with open(self._rejected_goal_path, 'w') as file:
                data = dict()
                data["goal_rejected"] = True
                yaml.dump(data, file)

        data = None
        if os.path.isfile(self._planning_time_path):
            with open(self._planning_time_path, 'r') as file_r:
                data = yaml.safe_load(file_r)
        else:
            data = dict()

        with open(self._planning_time_path, 'w') as file_w:
            data[str(self._planning_attempt_index)] = -1
            yaml.dump(data, file_w)

        raise ExternalShutdownException
        
def main(args = None):
    try:
        rclpy.init(args=args)
        action_client = SendNewGoalNode()
        action_client.send_goal()
        rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        rclpy.shutdown()
        action_client.destroy_node()

if __name__ == "__main__":
    main()