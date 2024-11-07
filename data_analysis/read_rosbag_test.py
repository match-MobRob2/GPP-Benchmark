import os
import math

from typing import List

from pathlib import Path as FilePath

import rclpy

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point

import matplotlib.pyplot as plt


class RosBagDataExtractor():
    def __init__(self, path:str):
        self._bagpath = FilePath('/home/rosjaeger/rosbag_data/dataset_101/rosbag_0')
        self.path: Path = None

    def get_path(self) -> None:
        # Create a type store to use if the bag has no message definitions.
        typestore = get_typestore(Stores.ROS2_HUMBLE)

        # Create reader instance and open for reading.
        with AnyReader([self._bagpath], default_typestore=typestore) as reader:
            connections = [x for x in reader.connections if x.topic == '/plan']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                self.path = reader.deserialize(rawdata, connection.msgtype)

class DataEvaluator():
    def __init__(self, rosbag_path_list: List[str]):
        self._rosbag_path_list: List[str] = rosbag_path_list
        self._rosbag_data_extractor: List[RosBagDataExtractor] = list()

    def create_data_extractor(self) -> None:
        for rosbag_path in self._rosbag_path_list:
            data_extractor: RosBagDataExtractor = RosBagDataExtractor(rosbag_path)
            data_extractor.get_path()
            self._rosbag_data_extractor.append(data_extractor)

    def calc_path_length(self) -> List[float]:
        path_length_list: List[float] = list()

        for index, rosbag_data_extractor in enumerate(self._rosbag_data_extractor):
            path: Path = rosbag_data_extractor.path
            
            last_pose: PoseStamped = None
            path_length: float = 0.0
            for pose in path.poses:
                if last_pose is None:
                    last_pose = pose
                    continue

                y_diff:float = pose.position.y - last_pose.position.y
                x_diff:float = pose.position.x - last_pose.position.x
                segment_length: float = math.sqrt(math.pow(y_diff, 2) + math.pow(x_diff, 2))

                path_length = path_length + segment_length
                last_pose = pose

            path_length_list.append(path_length)

        return path_length_list
    
    def plot_path_length(self) -> None:
        path_length_list: List[float] = self.calc_path_length()
        

            
if __name__ == "__main__":
    path_to_dataset: str = "/home/rosjaeger/rosbag_data/dataset_101"

    # Get all rosbags in the dataset
    rosbag_names: List[str] = [folder_name for folder_name in os.listdir(path_to_dataset) if os.path.isdir(os.path.join(path_to_dataset, folder_name))]
    rosbag_names.sort() # Sort list as os.listdir function returns unsorted list
    absolut_rosbag_path_list: List[str] = [os.path.join(path_to_dataset, rosbag_name) for rosbag_name in rosbag_names]

    data_evaluator: DataEvaluator = DataEvaluator(absolut_rosbag_path_list)
    data_evaluator.create_data_extractor()
