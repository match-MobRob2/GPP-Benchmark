#!/usr/bin/env python3
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class SendNewGoalNode(Node):
    def __init__(self) -> None:
        super().__init__('send_new_goal')
        self._goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)

    def send_goal(self):
        goal_pose: PoseStamped = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = 5.0
        goal_pose.pose.position.y = 5.0
        goal_pose.pose.orientation.w = 1.0

        self._goal_pose_publisher.publish(goal_pose)

def main(args = None):
    # Initialize ROS2 node and ROS2 communication (e.g. topics)
    rclpy.init(args = args)

    send_new_goal_node: SendNewGoalNode = SendNewGoalNode()

    # Publisher will only be called once. It needs time to be ready.
    sleep(1)

    send_new_goal_node.send_goal()

    # Pause the programm until its killed. All tasks still will be processed in the background.
    # rclpy.spin(send_new_goal_node)

    # Destroy the node explicitly, otherwise garbage collector should do it.
    send_new_goal_node.destroy_node()

    # Stops all communication and should always be called last in a node before finishing.
    rclpy.shutdown()

if __name__ == "__main__":
    main()