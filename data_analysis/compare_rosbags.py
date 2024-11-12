import os
import math
import yaml

from typing import List, Dict

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
        self.path_length: float = 0.0
        self.planning_time: float = 0.0

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

    def calc_path_length(self) -> float: 
        if self.path is None:
            self.path_length = -1.0
            return self.path_length
        
        last_pose: PoseStamped = None
        pose: PoseStamped = None
        self.path_length = 0.0
        for pose in self.path.poses:                
            if last_pose is None:
                last_pose = pose
                continue

            y_diff:float = pose.pose.position.y - last_pose.pose.position.y
            x_diff:float = pose.pose.position.x - last_pose.pose.position.x
            segment_length: float = math.sqrt(math.pow(y_diff, 2) + math.pow(x_diff, 2))

            self.path_length = self.path_length + segment_length
            last_pose = pose
        
        return self.path_length

class DataEvaluator():
    def __init__(self, dataset_path_list: Dict[str, str]):
        self._dataset_path_list: Dict[str,str] = dataset_path_list
        
        self._rosbag_data_extractor: Dict[str, List[RosBagDataExtractor]] = dict()

    ### Data preparation methods

    def create_data_extractor(self) -> None:
        for key, dataset_path in self._dataset_path_list.items():
            rosbag_names: List[str] = [folder_name for folder_name in os.listdir(dataset_path) if os.path.isdir(os.path.join(dataset_path, folder_name))]
            rosbag_names.sort() # Sort list as os.listdir function returns unsorted list
            absolut_rosbag_path_list: List[str] = [os.path.join(dataset_path, rosbag_name) for rosbag_name in rosbag_names]
            rosbag_extractor_list = list()
            for rosbag_path in absolut_rosbag_path_list:
                data_extractor: RosBagDataExtractor = RosBagDataExtractor(rosbag_path)
                data_extractor.read_rosbag()
                rosbag_extractor_list.append(data_extractor)
            self._rosbag_data_extractor[key] = rosbag_extractor_list

    def read_planning_time_data(self) -> None:
        for key, dataset_path in self._dataset_path_list.items():
            yaml_file_path: str = os.path.join(dataset_path, "planning_time.yaml")
            with open(yaml_file_path, 'r') as file:
                config_data = yaml.safe_load(file)
                for counter in range(len(config_data.items())):
                    print(config_data[str(counter)])
                    self._rosbag_data_extractor[key][counter].planning_time = round(config_data[str(counter)] * math.pow(10, -6), 2)


    ### Calculation methods

    def calc_path_length(self) -> None:
        path_length_list: List[float] = list()

        for key, value in self._rosbag_data_extractor.items():
            for rosbag_extractor in value:
                rosbag_extractor.calc_path_length()
    
    def get_path_length(self) -> Dict[str, List[float]]:
        path_length_dict: Dict[str, List[float]] = dict()
        for key, value in self._rosbag_data_extractor.items():
            path_length_list: List[float] = [extractor.path_length for extractor in value]
            path_length_dict[key] = path_length_list

        return path_length_dict
    
    def get_planning_time(self) -> Dict[str, List[float]]:
        planning_time_dict: Dict[str, List[float]] = dict()
        for key, value in self._rosbag_data_extractor.items():
            path_length_list: List[float] = [extractor.planning_time for extractor in value]
            planning_time_dict[key] = path_length_list

        return planning_time_dict

    ### Plotting methods
    
    def plot_path_length(self) -> None:
        path_length_list: Dict[str, List[float]] = self.get_path_length()

        for key, value in path_length_list.items():
            plt.plot(value)
            plt.xlabel(key)
            plt.ylabel("path length [m]")
            plt.show()

    def plot_compare_path_length(self) -> None:
        path_length_list: Dict[str, List[float]] = self.get_path_length()

        for key, value in path_length_list.items():
            plt.plot(value, label=key, marker="o", linestyle="dotted")

        plt.xlabel("index")
        plt.ylabel("path length [m]")
        plt.legend()
        plt.show()

    def plot_compare_planning_time(self) -> None:
        planning_time_list: Dict[str, List[float]] = self.get_planning_time()

        for key, value in planning_time_list.items():
            plt.plot(value, label=key, marker="o", linestyle="dotted")

        plt.xlabel("index")
        plt.ylabel("planning time [us]")
        plt.legend()
        plt.show()
            
if __name__ == "__main__":
    dataset_path_list: Dict[str,str] = {"PathPlanner1": "/home/lurz-match/rosbag_data/dataset_32", 
                                        "PathPlanner2": "/home/lurz-match/rosbag_data/dataset_33"}

    # Get all rosbags in the dataset
    # rosbag_names: List[str] = [folder_name for folder_name in os.listdir(path_to_dataset) if os.path.isdir(os.path.join(path_to_dataset, folder_name))]
    # rosbag_names.sort() # Sort list as os.listdir function returns unsorted list
    # absolut_rosbag_path_list: List[str] = [os.path.join(path_to_dataset, rosbag_name) for rosbag_name in rosbag_names]

    data_evaluator: DataEvaluator = DataEvaluator(dataset_path_list)
    data_evaluator.create_data_extractor()
    # data_evaluator.calc_path_length()
    data_evaluator.read_planning_time_data()
    # data_evaluator.plot_compare_path_length()
    data_evaluator.plot_compare_planning_time()

