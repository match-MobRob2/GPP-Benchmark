

#!/usr/bin/env python3
import os
import yaml

from typing import Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

class CreateNewPositionListNode(Node):
    def __init__(self) -> None:
        super().__init__('create_new_position_list')
        # self._goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.subscription = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.global_costmap_cb, 10)

        self._costmap: OccupancyGrid = None

    def global_costmap_cb(self, costmap_msg: OccupancyGrid) -> None:
        self._costmap = costmap_msg

    def generate_random_position(self) -> Dict:
        pass


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

    send_new_goal_node: CreateNewPositionListNode = CreateNewPositionListNode()

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





class PipelineConfig:
    map_name: str
    map_package: str

    robot_launch_file: str
    robot_launch_package: str

    number_of_tests: int

    robot_spawn_position_x: float
    robot_spawn_position_y: float
    robot_spawn_orientation_yaw: float

    robot_target_position_x: float
    robot_target_position_y: float
    robot_target_orientation_yaw: float

    def __init__(self) -> None:
        pass
    
    def import_config(self) -> None:

        # Get directory the script is in
        file_dir: str = os.path.dirname(os.path.realpath(__file__))
        # Get file path but with /../ in the middle
        file_name: str = os.path.join(file_dir, '../config/config.yaml')
        # Get absolut path without /../ in the middle
        config_path:str = os.path.abspath(os.path.realpath(file_name))

        with open(config_path, 'r') as file:
            config_data = yaml.safe_load(file)
            
            self.map_name = config_data["map_name"]
            self.map_package = config_data["map_package"]

            self.robot_launch_file = config_data["robot_launch_file"]
            self.robot_launch_package = config_data["robot_launch_package"]

            self.number_of_tests = config_data["number_of_tests"]

            self.robot_spawn_position_x = config_data["robot_spawn_position_x"]
            self.robot_spawn_position_y = config_data["robot_spawn_position_y"]
            self.robot_spawn_orientation_yaw = config_data["robot_spawn_orientation_yaw"]

            self.robot_target_position_x = config_data["robot_target_position_x"]
            self.robot_target_position_y = config_data["robot_target_position_y"]
            self.robot_target_orientation_yaw = config_data["robot_target_orientation_yaw"]



if __name__=="__main__":
    pass