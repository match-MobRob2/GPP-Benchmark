

#!/usr/bin/env python3
import os
import yaml
import random
from math import pi

from typing import Dict

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import rclpy.time
from visualization_msgs.msg import Marker, MarkerArray
from typing import List

class CreatePositionListNode(Node):
    def __init__(self) -> None:
        super().__init__('create_new_position_list')

        # Testing:
        self._marker_publisher = self.create_publisher(Marker, '/testing_marker', 1)
        self._marker_array_publisher = self.create_publisher(MarkerArray, '/testing_marker_array', 1)

    def run(self) -> None:
        # Read position data from file
        position_data: Dict[str, Dict[str, Dict[str, float]]]
        with open("/home/lurz-match/Desktop/position.yaml", 'r', encoding="utf-8") as file:
            position_data = yaml.safe_load(file)

        position_list: List[Dict[str, Dict[str, float]]] = list(position_data.values())

        marker_array: MarkerArray = MarkerArray()
        counter = 0

        for experiment_counter in range(len(position_data.items())):
            print("Start run: " + str(experiment_counter))
            
            current_position: Dict[str, Dict[str, float]] = position_list[experiment_counter]
            start_position: Dict[str, float] = current_position["start_position"]
            target_position: Dict[str, float] = current_position["target_position"]

            # Add start_position to the marker array
            start_marker: Marker = Marker()
            start_marker.header.frame_id = "map"
            start_marker.header.stamp = rclpy.time.Time().to_msg()
            start_marker.id = counter
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position.x = start_position["x"]
            start_marker.pose.position.y = start_position["y"]
            start_marker.pose.position.z = 0.0
            start_marker.pose.orientation.x = 0.0
            start_marker.pose.orientation.y = 0.0
            start_marker.pose.orientation.z = 0.0
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = 0.3
            start_marker.scale.y = 0.3
            start_marker.scale.z = 0.3
            start_marker.color.a = 1.0
            start_marker.color.r = 1.0
            marker_array.markers.append(start_marker)
            counter = counter + 1

            # Add target_position to the marker array
            target_marker: Marker = Marker()
            target_marker.header.frame_id = "map"
            target_marker.header.stamp = rclpy.time.Time().to_msg()
            target_marker.id = counter
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = target_position["x"]
            target_marker.pose.position.y = target_position["y"]
            target_marker.pose.position.z = 0.0
            target_marker.pose.orientation.x = 0.0
            target_marker.pose.orientation.y = 0.0
            target_marker.pose.orientation.z = 0.0
            target_marker.pose.orientation.w = 1.0
            target_marker.scale.x = 0.3
            target_marker.scale.y = 0.3
            target_marker.scale.z = 0.3
            target_marker.color.a = 1.0
            target_marker.color.g = 1.0
            marker_array.markers.append(target_marker)
            counter = counter + 1

        self._marker_array_publisher.publish(marker_array)

def main(args = None):
    # Initialize ROS2 node and ROS2 communication (e.g. topics)
    rclpy.init(args = args)

    create_position_list_node: CreatePositionListNode = CreatePositionListNode()

    # rclpy.spin(create_position_list_node)
    create_position_list_node.run()

    # Destroy the node explicitly, otherwise garbage collector should do it.
    create_position_list_node.destroy_node()

    # Stops all communication and should always be called last in a node before finishing.
    rclpy.shutdown()

if __name__ == "__main__":
    main()
