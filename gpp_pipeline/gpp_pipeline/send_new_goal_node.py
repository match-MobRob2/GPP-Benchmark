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
    # def __init__(self) -> None:
    #     super().__init__('send_new_goal')
        
    #     self._goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
    #     self._goal_pose_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_cb, 10)

        

    #     self.get_logger().info("subscribers: " + str(self._goal_pose_publisher.get_subscription_count()))
    #     while self._goal_pose_publisher.get_subscription_count() == 0:
    #         # rclpy.spin_once(self)
    #         self.get_logger().info("subscribers: " + str(self._goal_pose_publisher.get_subscription_count()))

    #     # self.get_logger().info("subscribers: " + str(self._goal_pose_publisher.get_subscription_count()))
    #     self.declare_parameter('target_robot_x', rclpy.Parameter.Type.DOUBLE)
    #     self.declare_parameter('target_robot_y', rclpy.Parameter.Type.DOUBLE)
    #     self.declare_parameter('target_robot_phi', rclpy.Parameter.Type.DOUBLE)

    #     self._target_robot_x: float = self.get_parameter("target_robot_x").value
    #     self._target_robot_y: float = self.get_parameter("target_robot_y").value
    #     self._target_robot_phi: float = self.get_parameter("target_robot_phi").value

    # def send_goal_cb(self) -> None:
    #     if self._goal_pose_publisher.get_subscription_count() < 1:
    #         self.get_logger().info("return")
    #         return
        
    #     goal_pose: PoseStamped = PoseStamped()
    #     goal_pose.header.frame_id = "map"
    #     goal_pose.header.stamp  = rclpy.time.Time().to_msg()
    #     goal_pose.pose.position.x = self._target_robot_x
    #     goal_pose.pose.position.y = self._target_robot_y
    #     goal_pose.pose.position.z = 0.0
    #     goal_pose.pose.orientation = quaternion_from_euler(0.0, 0.0, self._target_robot_phi)

    #     self._goal_pose_publisher.publish(goal_pose)
    #     self.get_logger().info("published")

    # def goal_pose_cb(self, goal_pose: PoseStamped) -> None:
    #     self.get_logger().info("goal pose received")
    #     print(goal_pose)

   

    # def run(self) -> None:
    #     goal_pose: PoseStamped = PoseStamped()
    #     goal_pose.header.frame_id = "map"
    #     # goal_pose.header.stamp  = rclpy.time.Time().to_msg()
    #     goal_pose.header.stamp  = rclpy.clock.Clock().now().to_msg()
    #     goal_pose.pose.position.x = self._target_robot_x
    #     goal_pose.pose.position.y = self._target_robot_y
    #     goal_pose.pose.position.z = 0.0
    #     tf_quaternion = quaternion_from_euler(0.0, 0.0, self._target_robot_phi)
    #     goal_pose.pose.orientation.x = tf_quaternion[0]
    #     goal_pose.pose.orientation.y = tf_quaternion[1]
    #     goal_pose.pose.orientation.z = tf_quaternion[2]
    #     goal_pose.pose.orientation.w = tf_quaternion[3]
        
    #     self._goal_pose_publisher.publish(goal_pose)
    #     self.get_logger().info("published")

    #     # file_path: str = os.path.join(os.path.expanduser("~"), "Desktop/position.yaml")
    #     # with open(file_path, 'r', encoding="utf-8") as file:
    #     #     position_data: Dict[str, Dict[str, Dict[str, float]]] = yaml.safe_load(file)

    #     #     for key, position_tuple in position_data.items():
    #     #         goal_pose: PoseStamped = PoseStamped()
    #     #         goal_pose.header.frame_id = "map"
    #     #         goal_pose.pose.position.x = position_tuple["target_position"]["x"]
    #     #         goal_pose.pose.position.y = position_tuple["target_position"]["y"]
    #     #         goal_pose.pose.orientation.w = 1.0

    #     #         self.get_logger().info(key)

    #     #         self._goal_pose_publisher.publish(goal_pose)
    #     #         self.get_logger().info(key)
    #     #         # rclpy.spin_once(self)
    #     #         self.get_logger().info(key)
    #     #         sleep(1.0)
    #     #         self.get_logger().info(key)


    # def __init__(self) -> None:
    #     super().__init__('send_new_goal')
    #     self._action_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

    # def send_goal(self):
    #     self.get_logger().info('Waiting for action server...')
    #     self._action_client.wait_for_server()

    #     goal_msg = ComputePathToPose.Goal()
    #     goal_msg

    #     self.get_logger().info('Sending goal request...')

    #     self._send_goal_future = self._action_client.send_goal_async(
    #         goal_msg,
    #         feedback_callback=self.feedback_callback)

    #     self._send_goal_future.add_done_callback(self.goal_response_callback)

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
        self.get_logger().info(str(feedback.feedback.current_pose))
        self.get_logger().info("feedback")
        # self.get_logger().info('Received feedback: {0}'.format(str(feedback.navigation_time.nanosec)))

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

        self._send_goal_future.add_done_callback(self.goal_response_callback)


            

def main(args = None):
    # Initialize ROS2 node and ROS2 communication (e.g. topics)
    # rclpy.init(args = args)

    # send_new_goal_node: SendNewGoalNode = SendNewGoalNode()

    # Publisher will only be called once. It needs time to be ready.
    # sleep(1)

    try:
        rclpy.init(args=args)
        
        action_client = SendNewGoalNode()

        action_client.send_goal()

        rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    # Pause the programm until its killed. All tasks still will be processed in the background.
    # rclpy.spin(send_new_goal_node)

    # Destroy the node explicitly, otherwise garbage collector should do it.
    # send_new_goal_node.destroy_node()

    # Stops all communication and should always be called last in a node before finishing.
    # rclpy.shutdown()

if __name__ == "__main__":
    main()