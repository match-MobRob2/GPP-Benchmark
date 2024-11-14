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

            connections = [x for x in reader.connections if x.topic == '/global_costmap/costmap']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                self.costmap = reader.deserialize(rawdata, connection.msgtype)

            connections = [x for x in reader.connections if x.topic == '/map']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                self.map = reader.deserialize(rawdata, connection.msgtype)

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
                    self._rosbag_data_extractor[key][counter].planning_time = round(config_data[str(counter)] * math.pow(10, -6), 2)


    ### Calculation methods

    def calc_path_length(self) -> None:
        path_length_list: List[float] = list()

        for key, value in self._rosbag_data_extractor.items():
            for rosbag_extractor in value:
                rosbag_extractor.calc_path_length()
    
    def calc_planning_time_mean(self) -> List[float]:
        number_of_tests: int = len(next(iter(self._rosbag_data_extractor.values())))
        mean_list: List[float] = [0 for i in range(number_of_tests)]

        for key, value in self._rosbag_data_extractor.items():
            for index, rosbag_extractor in enumerate(value):
                mean_list[index] = mean_list[index] + rosbag_extractor.planning_time / float(len(self._rosbag_data_extractor))

        return mean_list

    def calc_planning_time_error(self) -> tuple[List[float]]:
        number_of_tests: int = len(next(iter(self._rosbag_data_extractor.values())))
        min_value_list: List[float] = [None for i in range(number_of_tests)]
        max_value_list: List[float] = [None for i in range(number_of_tests)]

        for key, value in self._rosbag_data_extractor.items():
            for index, rosbag_extractor in enumerate(value):
                if min_value_list[index] is None or \
                    min_value_list[index] > rosbag_extractor.planning_time:
                    min_value_list[index] = rosbag_extractor.planning_time
                
                if max_value_list[index] is None or \
                    max_value_list[index] < rosbag_extractor.planning_time:
                    max_value_list[index] = rosbag_extractor.planning_time

        mean_value_list: List[float] = self.calc_planning_time_mean()

        neg_error: List[float] = [mean_value - min_value for mean_value, min_value in zip(mean_value_list, min_value_list)]
        pos_error: List[float] = [max_value -mean_value for mean_value, max_value in zip(mean_value_list, max_value_list)]

        return (neg_error, pos_error)
                
    def calc_planning_time_max_error(self) -> List[float]:
        number_of_tests: int = len(next(iter(self._rosbag_data_extractor.values())))
        max_error_list: List[float] = [None for i in range(number_of_tests)]

        mean_value_list: List[float] = self.calc_planning_time_mean()

        for key, value in self._rosbag_data_extractor.items():
            for index, rosbag_extractor in enumerate(value):
                diff: float = abs(mean_value_list[index] - rosbag_extractor.planning_time)
                if max_error_list[index] is None or \
                    max_error_list[index] < diff:
                    max_error_list[index] = diff

        return max_error_list
                

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

        figsize_inches = (83 / 25.4, 50 / 25.4)
        plt.figure(figsize=figsize_inches)
        plt.rcParams.update({'font.size': 10})
        plt.rcParams['font.family'] = 'Times New Roman'

        path_length = next(iter(path_length_list.values()))
        index_list: List[int] = range(len(path_length))

        # for key, value in path_length_list.items():
        #     plt.plot(value, label=key, marker="o", linestyle="None", color="#018571")
        plt.bar(index_list, path_length, width=1.0, label="Path Length 1-5", color="#018571")

        plt.xlabel("Index")
        plt.ylabel("Path Length [m]")
        # plt.legend()

        plt.savefig("/home/lurz-match/Desktop/test.png", format='png', bbox_inches='tight')

        plt.show()

    def plot_compare_planning_time(self) -> None:
        planning_time_list: Dict[str, List[float]] = self.get_planning_time()

        for key, value in planning_time_list.items():
            plt.plot(value, label=key, marker="o", linestyle="dotted")

        plt.xlabel("index")
        plt.ylabel("planning time [us]")
        plt.legend()
        plt.show()

    def plot_planning_time_with_diff(self) -> None:
        mean_value_list: List[float] = self.calc_planning_time_mean()
        error_bar: tuple[List[float]] = self.calc_planning_time_error()

        index_list: List[int] = range(len(mean_value_list))

        plt.figure(figsize=(8, 6))
        plt.errorbar(index_list, mean_value_list, yerr=error_bar, fmt='o', ecolor='red', capsize=5, capthick=1, color='blue', label="Data with error bars")

        plt.xlim(-3, 103)  # Set x-axis from 2 to 8
        plt.ylim(0, 100)

        # Set titles and labels
        # plt.title(title)
        plt.xlabel("index")
        plt.ylabel("planning time [us]")
        plt.legend()

        # Show the plot
        plt.show()

    def plot_planning_time_and_diff(self) -> None:
        mean_value_list: List[float] = self.calc_planning_time_mean()
        max_error_list: List[float] = self.calc_planning_time_max_error()

        mean_error: float = 0.0
        for value in max_error_list:
            mean_error = mean_error + value / len(max_error_list)

        mean_planning_time: float = 0.0
        for value in mean_value_list:
            mean_planning_time = mean_planning_time + value / len(mean_value_list)

        # plt.plot(mean_value_list, label=key, marker="o", linestyle="dotted")
        group_values = ["mean planning time", "max error"]
        index_list: List[int] = range(len(mean_value_list))

        # for i, (group_name, color) in enumerate(zip(group_values, ['blue', 'red'])):
            # Extract values for each group across all indices

        figsize_inches = (83 / 25.4, 50 / 25.4)
        plt.figure(figsize=figsize_inches)
        plt.rcParams.update({'font.size': 10})
        plt.rcParams['font.family'] = 'Times New Roman'

        plt.bar(index_list, mean_value_list, width=1.0, label="Mean Planning Time", color="#018571")
        plt.bar([i for i in index_list], max_error_list, width=1.0, label="Max Time Difference", color="#a6611a")
        # plt.plot([mean_error for i in range(len(max_error_list))], color="#FF00FF", linestyle="dashed", label="average max error")
        # plt.plot([mean_planning_time for i in range(len(max_error_list))], color="#30D5C8", linestyle="dashed", label="overall average planning time")


        plt.xlim(-5, 105)  # Set x-axis from 2 to 8
        plt.ylim(0, 110)

        plt.xlabel("Index")
        plt.ylabel("Planning time [ms]")
        plt.legend()
        
        plt.savefig("/home/lurz-match/Desktop/test.png", format='png', bbox_inches='tight')

        plt.show()
 
    def plot_compare_planning_time_specific(self) -> None:
        planning_time_list: Dict[str, List[float]] = self.get_planning_time()

        for key, value in planning_time_list.items():
            linestyle: str = None
            marker: str = "o"
            color:str = None

            if "Desktop" in key:
                color="red"
            elif "Cluster" in key:
                color="blue"
            elif "Laptop" in key:
                color="green"

            if "NavFN" in key:
                linestyle="dotted"
            elif "Smac" in key:
                linestyle="dashed"
            elif "ThetaStar" in key:
                linestyle="dashdot"

            plt.plot(value, label=key,color=color, marker=marker, linestyle=linestyle)

        plt.xlabel("index")
        plt.ylabel("planning time [us]")
        plt.legend()
        plt.show()

    def plot_costmap(self) -> None:
        figsize_inches = (83 / 25.4, 50 / 25.4)
        plt.figure(figsize=figsize_inches)
        plt.rcParams.update({'font.size': 10})
        plt.rcParams['font.family'] = 'Times New Roman'
        
        first_extractor:RosBagDataExtractor = next(iter(self._rosbag_data_extractor.values()))[0]
        cost_map = [first_extractor.map.data[i:i + first_extractor.map.info.width] for i in range(0, len(first_extractor.map.data), first_extractor.map.info.width)]
        
        plt.imshow(cost_map, cmap='binary', interpolation='nearest')

        for extractor in next(iter(self._rosbag_data_extractor.values())):
            x_values: List[float] = list()
            y_values: List[float] = list()
            for pose in extractor.path.poses:
                x_values.append((pose.pose.position.x / first_extractor.map.info.resolution) - (first_extractor.map.info.origin.position.x / first_extractor.map.info.resolution))
                y_values.append((pose.pose.position.y / first_extractor.map.info.resolution) - (first_extractor.map.info.origin.position.y / first_extractor.map.info.resolution))

            plt.plot(x_values, y_values, color='#018571', linestyle='-', marker='')

        # Add labels and title
        plt.title("2D Cost Map")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        
        plt.savefig("/home/lurz-match/Desktop/test.png", format='png', bbox_inches='tight')

        # Show the plot
        plt.show()

if __name__ == "__main__":
    # dataset_path_list: Dict[str,str] = {"Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_1/dataset_18", 
    #                                     "Cluster - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/Smac/dataset_17",
    #                                     "Cluster - ThetaStar": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/ThetaStar/dataset_19"}

    # dataset_path_list: Dict[str,str] = {"Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_1/dataset_18", 
    #                                     "Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_2/dataset_18",
    #                                     "Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_3/dataset_18",
    #                                     "Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_4/dataset_18",
    #                                     "Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_5/dataset_18"}

    dataset_path_list: Dict[str,str] = {"Cluster - NavFN - c1": "/home/lurz-match/Desktop/CirpDesign2025/Cluster/NavFN/cycle_1/dataset_22", 
                                        "Cluster - NavFN - c2": "/home/lurz-match/Desktop/CirpDesign2025/Cluster/NavFN/cycle_2/dataset_25",
                                        "Cluster - NavFN - c3": "/home/lurz-match/Desktop/CirpDesign2025/Cluster/NavFN/cycle_3/dataset_26",
                                        "Cluster - NavFN - c4": "/home/lurz-match/Desktop/CirpDesign2025/Cluster/NavFN/cycle_4/dataset_27",
                                        "Cluster - NavFN - c5": "/home/lurz-match/Desktop/CirpDesign2025/Cluster/NavFN/cycle_5/dataset_30"}

    # dataset_path_list: Dict[str,str] = {"Desktop - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/NavFN/dataset_170",
    #                                     "Desktop - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/Smac/dataset_171",
    #                                     "Desktop - ThetaStar": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/ThetaStar/dataset_169",
    #                                     "Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_1/dataset_22",
    #                                     "Cluster - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/Smac/dataset_23"}

    # Get all rosbags in the dataset
    # rosbag_names: List[str] = [folder_name for folder_name in os.listdir(path_to_dataset) if os.path.isdir(os.path.join(path_to_dataset, folder_name))]
    # rosbag_names.sort() # Sort list as os.listdir function returns unsorted list
    # absolut_rosbag_path_list: List[str] = [os.path.join(path_to_dataset, rosbag_name) for rosbag_name in rosbag_names]

    data_evaluator: DataEvaluator = DataEvaluator(dataset_path_list)
    data_evaluator.create_data_extractor()
    data_evaluator.calc_path_length()
    data_evaluator.read_planning_time_data()
    data_evaluator.plot_compare_path_length()
    # data_evaluator.plot_compare_planning_time()
    # data_evaluator.plot_compare_planning_time_specific()
    # data_evaluator.plot_planning_time_with_diff()
    # data_evaluator.plot_planning_time_and_diff()
    # data_evaluator.plot_costmap()

