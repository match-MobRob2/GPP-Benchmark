

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

class CreatePositionListNode(Node):
    def __init__(self) -> None:
        super().__init__('create_new_position_list')
        self.subscription = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.global_costmap_cb, 10)
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        # Testing:
        self._marker_publisher = self.create_publisher(Marker, '/testing_marker', 1)
        self._marker_array_publisher = self.create_publisher(MarkerArray, '/testing_marker_array', 1)

        self._costmap: OccupancyGrid = None
        self._map: OccupancyGrid = None

        # Get directory the script is in
        file_dir: str = os.path.dirname(os.path.realpath(__file__))
        self.get_logger().info(file_dir)

        # Get file path but with /../ in the middle
        file_name: str = os.path.join(file_dir, '../../../../share/gpp_pipeline/config/config.yaml')
        self.get_logger().info(file_name)
        # Get absolut path without /../ in the middle
        config_path:str = os.path.abspath(os.path.realpath(file_name))
        self.get_logger().info(config_path)

        with open(config_path, 'r', encoding="utf-8") as file:
            config_data = yaml.safe_load(file)
            self._number_of_tests = config_data["number_of_tests"]

    def global_costmap_cb(self, costmap_msg: OccupancyGrid) -> None:
        self._costmap = costmap_msg

    def map_cb(self, map_msg: OccupancyGrid) -> None:
        self._map = map_msg

    def generate_random_position(self) -> Dict[str, float]:
        # self.get_logger().info(str(type(self._costmap)))
        # self.get_logger().info(str(type(self._costmap.data)))
        # self.get_logger().info(str(len(self._costmap.data)))
        # self.get_logger().info(str(self._costmap.info.resolution))
        # self.get_logger().info(str(self._costmap.info.width))
        # self.get_logger().info(str(self._costmap.info.height))
        # self.get_logger().info(str(self._costmap.info.origin))
        
        is_valid: bool = False

        costmap_new_x: int = None
        costmap_new_y: int = None

        while not is_valid:
            self.get_logger().info("while")
            costmap_new_x = random.randint(0, self._costmap.info.width - 1)
            costmap_new_y = random.randint(0, self._costmap.info.height - 1)
            cell_index = costmap_new_y * self._costmap.info.width + costmap_new_x
            if self._costmap.data[cell_index] == 0 and \
                self._map != -1 and self._map != 100:
                is_valid = True
            else:
                self.get_logger().info("Position invalid")

        map_new_x: float = costmap_new_x * self._costmap.info.resolution + self._costmap.info.origin.position.x
        map_new_y: float = costmap_new_y * self._costmap.info.resolution + self._costmap.info.origin.position.y
        map_new_phi: float = round(random.uniform(-pi, pi), 3)

        self.get_logger().info("x: " + str(map_new_x))
        self.get_logger().info("y: " + str(map_new_y))

        new_position: Dict[str, float] = dict()
        new_position["x"] = map_new_x
        new_position["y"] = map_new_y
        new_position["phi"] = map_new_phi
        
        return new_position

    def generate_random_position_list(self) -> Dict[str, Dict[str, float]]:
        new_position_list: Dict[str, Dict[str, float]] = dict()
        for counter in range(0,self._number_of_tests):
            new_position: Dict[str, float] = self.generate_random_position()
            new_position_list["position_" + str(counter)] = new_position
            self.get_logger().info(str(counter))

        return new_position_list

    def run(self) -> None:
        while self._costmap is None and self._map is None:
            rclpy.spin_once(self)

        position_list: Dict[str, Dict[str, float]] = self.generate_random_position_list()

        marker_array: MarkerArray = MarkerArray()
        counter: int = 0
        for key, value in position_list.items():
            new_marker: Marker = Marker()
            new_marker.header.frame_id = "map"
            new_marker.header.stamp = rclpy.time.Time().to_msg()
            new_marker.id = counter
            new_marker.type = Marker.SPHERE
            new_marker.action = Marker.ADD
            new_marker.pose.position.x = value["x"]
            new_marker.pose.position.y = value["y"]
            new_marker.pose.position.z = 0.0
            new_marker.pose.orientation.x = 0.0
            new_marker.pose.orientation.y = 0.0
            new_marker.pose.orientation.z = 0.0
            new_marker.pose.orientation.w = 1.0
            new_marker.scale.x = 0.3
            new_marker.scale.y = 0.3
            new_marker.scale.z = 0.3
            new_marker.color.a = 1.0
            new_marker.color.r = 1.0
            marker_array.markers.append(new_marker)
            counter = counter + 1

        self._marker_array_publisher.publish(marker_array)


def main(args = None):
    # Initialize ROS2 node and ROS2 communication (e.g. topics)
    rclpy.init(args = args)

    create_position_list_node: CreatePositionListNode = CreatePositionListNode()

    # Publisher will only be called once. It needs time to be ready.
    # sleep(1)

    # send_new_goal_node.send_goal()

    # Pause the programm until its killed. All tasks still will be processed in the background.
    # rclpy.spin(send_new_goal_node)

    # Destroy the node explicitly, otherwise garbage collector should do it.
    # send_new_goal_node.destroy_node()

    # rclpy.spin(create_position_list_node)
    create_position_list_node.run()

    # Stops all communication and should always be called last in a node before finishing.
    rclpy.shutdown()

if __name__ == "__main__":
    main()





# class PipelineConfig:
#     map_name: str
#     map_package: str

#     robot_launch_file: str
#     robot_launch_package: str

#     number_of_tests: int

#     robot_spawn_position_x: float
#     robot_spawn_position_y: float
#     robot_spawn_orientation_yaw: float

#     robot_target_position_x: float
#     robot_target_position_y: float
#     robot_target_orientation_yaw: float

#     def __init__(self) -> None:
#         pass
    
#     def import_config(self) -> None:

#         # Get directory the script is in
#         file_dir: str = os.path.dirname(os.path.realpath(__file__))
#         # Get file path but with /../ in the middle
#         file_name: str = os.path.join(file_dir, '../config/config.yaml')
#         # Get absolut path without /../ in the middle
#         config_path:str = os.path.abspath(os.path.realpath(file_name))

#         with open(config_path, 'r') as file:
#             config_data = yaml.safe_load(file)
            
#             self.map_name = config_data["map_name"]
#             self.map_package = config_data["map_package"]

#             self.robot_launch_file = config_data["robot_launch_file"]
#             self.robot_launch_package = config_data["robot_launch_package"]

#             self.number_of_tests = config_data["number_of_tests"]

#             self.robot_spawn_position_x = config_data["robot_spawn_position_x"]
#             self.robot_spawn_position_y = config_data["robot_spawn_position_y"]
#             self.robot_spawn_orientation_yaw = config_data["robot_spawn_orientation_yaw"]

#             self.robot_target_position_x = config_data["robot_target_position_x"]
#             self.robot_target_position_y = config_data["robot_target_position_y"]
#             self.robot_target_orientation_yaw = config_data["robot_target_orientation_yaw"]



# if __name__=="__main__":
#     pass