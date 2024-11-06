#!/usr/bin/env python3
from typing import Dict

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

from time import sleep

import rclpy.time

class PathListener(Node):
    def __init__(self) -> None:
        super().__init__('send_new_goal')
        self._path_subscriber = self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self._timeout_timer = self.create_timer(20.0, self.timeout_cb)
        self.plan = None

    def plan_cb(self, plan: Path) -> None:
        sleep(3)
        self.plan = plan
        self.get_logger().error("PathListener: Path received")

    def timeout_cb(self) -> None:
        self.plan = Path()
        self.get_logger().error("PathListener: Timeout")

def main(args = None):
    # Initialize ROS2 node and ROS2 communication (e.g. topics)
    rclpy.init(args = args)

    path_listener_node: PathListener = PathListener()

    # Pause the programm until its killed. All tasks still will be processed in the background.
    # rclpy.spin(send_new_goal_node)
    
    while path_listener_node.plan is None and rclpy.ok():
        # path_listener_node.get_logger().info(str(path_listener_node.plan))
        # path_listener_node.get_logger().info(str(rclpy.ok()))
        # path_listener_node.get_logger().info("bla")
        rclpy.spin_once(path_listener_node)
        

    path_listener_node.get_logger().info("finished")


    # Destroy the node explicitly, otherwise garbage collector should do it.
    path_listener_node.destroy_node()

    # Stops all communication and should always be called last in a node before finishing.
    rclpy.shutdown()

if __name__ == "__main__":
    main()