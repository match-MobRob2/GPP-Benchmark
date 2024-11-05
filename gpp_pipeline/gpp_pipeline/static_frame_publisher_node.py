import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from time import sleep

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static robot frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):
        super().__init__('static_robot_tf_broadcaster')

        self.declare_parameter('start_robot_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('start_robot_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('start_robot_phi', rclpy.Parameter.Type.DOUBLE)

        self._start_robot_x: float = self.get_parameter("start_robot_x").value
        self._start_robot_y: float = self.get_parameter("start_robot_y").value
        self._start_robot_phi: float = self.get_parameter("start_robot_phi").value

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self._tf_pub_timer = self.create_timer(1.0, self.pub_transform_cb)
        self.get_logger().info("START ---------------------------------------------------")
        self.pub_transform_cb()
        self.get_logger().info("START ---------------------------------------------------")
        
        self.get_logger().info(str(self._tf_pub_timer.time_until_next_call()))
        self.get_logger().info(str(self._tf_pub_timer.timer_period_ns))
        self.get_logger().info(str(self._tf_pub_timer.is_ready()))
        self.get_logger().info(str(self._tf_pub_timer.time_until_next_call()))
        self.get_logger().info("START ---------------------------------------------------")


    def pub_transform_cb(self) -> None:
        self.get_logger().info("1---------------------------------------------------")
        # t = TransformStamped()

        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = "world"
        # t.child_frame_id = "base_link"

        # t.transform.translation.x = float(self._start_robot_x)
        # t.transform.translation.y = float(self._start_robot_y)
        # t.transform.translation.z = float(0.0)
        # quat = quaternion_from_euler(
        #     float(0.0), float(0.0), float(self._start_robot_phi))
        # t.transform.rotation.x = quat[0]
        # t.transform.rotation.y = quat[1]
        # t.transform.rotation.z = quat[2]
        # t.transform.rotation.w = quat[3]

        # self.tf_static_broadcaster.sendTransform(t)

        # self.get_logger().info("ALIVE---------------------------------------------------")


def main(args = None):
    # Initialize ROS2 node and ROS2 communication (e.g. topics)
    rclpy.init(args = args)

    static_robot_tf_publisher: StaticFramePublisher = StaticFramePublisher()

    # Publisher will only be called once. It needs time to be ready.
    sleep(1)

    # static_robot_tf_publisher.run()

    # Pause the programm until its killed. All tasks still will be processed in the background.
    rclpy.spin(static_robot_tf_publisher)

    # Destroy the node explicitly, otherwise garbage collector should do it.
    static_robot_tf_publisher.destroy_node()

    # Stops all communication and should always be called last in a node before finishing.
    rclpy.shutdown()
