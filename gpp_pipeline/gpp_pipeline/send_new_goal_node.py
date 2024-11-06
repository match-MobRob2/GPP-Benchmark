#!/usr/bin/env python3
import os
import yaml

from time import sleep
from typing import Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class SendNewGoalNode(Node):
    def __init__(self) -> None:
        super().__init__('send_new_goal')
        
        self._goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)

        self._send_goal_timer = self.create_timer(1.0, self.send_goal_cb)


        self.get_logger().info("subscribers: " + str(self._goal_pose_publisher.get_subscription_count()))
        # while self._goal_pose_publisher.get_subscription_count() < 2:
        #     # rclpy.spin_once(self)
        #     self.get_logger().info("subscribers: " + str(self._goal_pose_publisher.get_subscription_count()))

        # self.get_logger().info("subscribers: " + str(self._goal_pose_publisher.get_subscription_count()))
        self.declare_parameter('target_robot_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('target_robot_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('target_robot_phi', rclpy.Parameter.Type.DOUBLE)

        self._target_robot_x: float = self.get_parameter("target_robot_x").value
        self._target_robot_y: float = self.get_parameter("target_robot_y").value
        self._target_robot_phi: float = self.get_parameter("target_robot_phi").value

    def send_goal_cb(self) -> None:
        if self._goal_pose_publisher.get_subscription_count() < 2:
            self.get_logger().info("return")
            return
        
        goal_pose: PoseStamped = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = self._target_robot_x
        goal_pose.pose.position.y = self._target_robot_y
        goal_pose.pose.orientation.w = 1.0

        self._goal_pose_publisher.publish(goal_pose)
        self.get_logger().info("published")

    def run(self) -> None:
        goal_pose: PoseStamped = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = self._target_robot_x
        goal_pose.pose.position.y = self._target_robot_y
        goal_pose.pose.orientation.w = 1.0

        self._goal_pose_publisher.publish(goal_pose)
        self.get_logger().info("published")

        # file_path: str = os.path.join(os.path.expanduser("~"), "Desktop/position.yaml")
        # with open(file_path, 'r', encoding="utf-8") as file:
        #     position_data: Dict[str, Dict[str, Dict[str, float]]] = yaml.safe_load(file)

        #     for key, position_tuple in position_data.items():
        #         goal_pose: PoseStamped = PoseStamped()
        #         goal_pose.header.frame_id = "map"
        #         goal_pose.pose.position.x = position_tuple["target_position"]["x"]
        #         goal_pose.pose.position.y = position_tuple["target_position"]["y"]
        #         goal_pose.pose.orientation.w = 1.0

        #         self.get_logger().info(key)

        #         self._goal_pose_publisher.publish(goal_pose)
        #         self.get_logger().info(key)
        #         # rclpy.spin_once(self)
        #         self.get_logger().info(key)
        #         sleep(1.0)
        #         self.get_logger().info(key)

            

def main(args = None):
    # Initialize ROS2 node and ROS2 communication (e.g. topics)
    rclpy.init(args = args)

    send_new_goal_node: SendNewGoalNode = SendNewGoalNode()

    # Publisher will only be called once. It needs time to be ready.
    # sleep(3)

    # send_new_goal_node.run()

    # Pause the programm until its killed. All tasks still will be processed in the background.
    rclpy.spin(send_new_goal_node)

    # Destroy the node explicitly, otherwise garbage collector should do it.
    send_new_goal_node.destroy_node()

    # Stops all communication and should always be called last in a node before finishing.
    rclpy.shutdown()

if __name__ == "__main__":
    main()