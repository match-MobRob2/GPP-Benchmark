import os
import math
import yaml

from typing import List

from pathlib import Path as FilePath

import rclpy

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point

import matplotlib.pyplot as plt


class RosBagDataExtractor():
    def __init__(self, path: str):
        self._bagpath = FilePath(path)
        self.path: Path = None
        self.goal_pose: PoseStamped = None

    def read_rosbag(self) -> None:
        # Create a type store to use if the bag has no message definitions.
        typestore = get_typestore(Stores.ROS2_HUMBLE)

        # Create reader instance and open for reading.
        with AnyReader([self._bagpath], default_typestore=typestore) as reader:
            # connections = [x for x in reader.connections if x.topic == '/goal_pose']
            # for connection, timestamp, rawdata in reader.messages(connections=connections):
            #     self.goal_pose = reader.deserialize(rawdata, connection.msgtype)
            
            connections = [x for x in reader.connections if x.topic == '/plan']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                self.path = reader.deserialize(rawdata, connection.msgtype)

class DataEvaluator():
    def __init__(self, path_to_dataset: str, rosbag_path_list: List[str]):
        self._path_to_dataset: str = path_to_dataset
        self._rosbag_path_list: List[str] = rosbag_path_list
        self._rosbag_data_extractor: List[RosBagDataExtractor] = list()

    ### Data preparation methods

    def create_data_extractor(self) -> None:
        for rosbag_path in self._rosbag_path_list:
            data_extractor: RosBagDataExtractor = RosBagDataExtractor(rosbag_path)
            data_extractor.read_rosbag()
            self._rosbag_data_extractor.append(data_extractor)

    def read_planning_time_data(self) -> None:
        yaml_file_path: str = os.path.join(self._path_to_dataset, "planning_time.yaml")
        with open(yaml_file_path, 'r') as file:
            config_data = yaml.safe_load(file)
            self._planning_time_data = list()
            for key, value in config_data.items():
                self._planning_time_data.append(round(value * math.pow(10, -6), 2))

        print(self._planning_time_data)
        print(len(self._planning_time_data))

    ### Calculation methods

    def calc_path_length(self) -> List[float]:
        path_length_list: List[float] = list()

        for rosbag_data_extractor in self._rosbag_data_extractor:
            if rosbag_data_extractor.path is None:
                print("Path is None")
                path_length_list.append(-1)
                continue

            path: Path = rosbag_data_extractor.path
            
            last_pose: PoseStamped = None
            pose: PoseStamped = None
            path_length: float = 0.0
            for pose in path.poses:                
                if last_pose is None:
                    last_pose = pose
                    continue

                y_diff:float = pose.pose.position.y - last_pose.pose.position.y
                x_diff:float = pose.pose.position.x - last_pose.pose.position.x
                segment_length: float = math.sqrt(math.pow(y_diff, 2) + math.pow(x_diff, 2))

                path_length = path_length + segment_length
                last_pose = pose

            path_length_list.append(path_length)

        return path_length_list
    
    def calc_planning_time(self) -> List[float]:
        planning_time_list: List[float] = list()

        for rosbag_data_extractor in self._rosbag_data_extractor:
            if rosbag_data_extractor.goal_pose is not None and \
                rosbag_data_extractor.path is not None:

                goal_pose_time = rosbag_data_extractor.goal_pose.header.stamp
                path_time = rosbag_data_extractor.path.header.stamp

                # In Milliseconds
                time_diff = float((path_time.sec * math.pow(10, 9) - goal_pose_time.sec * math.pow(10, 9) 
                            + path_time.nanosec - goal_pose_time.nanosec) / (10*math.pow(10,6)))

                planning_time_list.append(time_diff)

            else:
                # planning_time_list.append(-1)
                pass

        return planning_time_list

    ### Plotting methods
    
    def plot_path_length(self) -> None:
        path_length_list: List[float] = self.calc_path_length()

        plt.plot(path_length_list)
        plt.xlabel("Index")
        plt.ylabel("path length [m]")
        plt.show()

    def plot_path_length(self) -> None:
        planning_time_list: List[float] = self.calc_planning_time()

        plt.plot(planning_time_list)
        plt.xlabel("Index")
        plt.ylabel("planning time [us]")
        plt.show()
            
if __name__ == "__main__":
    path_to_dataset: str = "/home/lurz-match/rosbag_data/dataset_32"

    # Get all rosbags in the dataset
    rosbag_names: List[str] = [folder_name for folder_name in os.listdir(path_to_dataset) if os.path.isdir(os.path.join(path_to_dataset, folder_name))]
    rosbag_names.sort() # Sort list as os.listdir function returns unsorted list
    absolut_rosbag_path_list: List[str] = [os.path.join(path_to_dataset, rosbag_name) for rosbag_name in rosbag_names]

    data_evaluator: DataEvaluator = DataEvaluator(path_to_dataset, absolut_rosbag_path_list)
    data_evaluator.create_data_extractor()
    data_evaluator.read_planning_time_data()
    # data_evaluator.plot_path_length()
    # data_evaluator.plot_path_length()
