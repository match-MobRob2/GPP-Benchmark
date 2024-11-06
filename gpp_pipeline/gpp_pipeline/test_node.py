#!/usr/bin/env python3

import rclpy
import rclpy.clock
from rclpy.node import Node
import rclpy.time

class TestNode(Node):
    def __init__(self) -> None:
        super().__init__('test_node')
        self.get_logger().info(str(rclpy.time.Time().nanoseconds))
        self._test_timer = self.create_timer(1.0, self.test_timer_cb, clock=rclpy.clock.Clock())
        self.get_logger().info(str(rclpy.time.Time().nanoseconds))

        self.get_logger().info("Timer created")
        self.get_logger().info(str(rclpy.time.Time().nanoseconds))

    def test_timer_cb(self) -> None:
        self.get_logger().info("Timer is working")

def main(args = None):
    # Initialize ROS2 node and ROS2 communication (e.g. topics)
    rclpy.init(args = args)

    test_node: TestNode = TestNode()

    # Publisher will only be called once. It needs time to be ready.
    # sleep(3)

    # Pause the programm until its killed. All tasks still will be processed in the background.
    rclpy.spin(test_node)

    # Destroy the node explicitly, otherwise garbage collector should do it.
    test_node.destroy_node()

    # Stops all communication and should always be called last in a node before finishing.
    rclpy.shutdown()

if __name__ == "__main__":
    main()