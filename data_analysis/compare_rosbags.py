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
import matplotlib.font_manager as font_manager


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
            self.path_length = 0.0
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
            for index, value in enumerate(path_length_list):
                if value > 1000:
                    path_length_list[index] = 0
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

    def plot_path_length(self) -> None:
        path_length_list: Dict[str, List[float]] = self.get_path_length()

        # figsize_inches = ((2*88) / 25.4, (2*50) / 25.4)
        figsize_inches = (88 / 25.4, 50 / 25.4)
        plt.figure(figsize=figsize_inches)
        plt.rcParams.update({'font.size': 7.8})
        plt.rcParams['font.family'] = 'serif'
        plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
        # plt.rcParams['font.family'] = 'Times New Roman'

        path_length = next(iter(path_length_list.values()))
        index_list: List[int] = range(len(path_length))

        # for key, value in path_length_list.items():
        #     plt.plot(value, label=key, marker="o", linestyle="None", color="#018571")
        plt.bar(index_list, path_length, width=1.0, label="Path Length 1-5", color="#018571")

        plt.xlim(-3, 103)  # Set x-axis from 2 to 8
        plt.ylim(0, 50)

        plt.xlabel("Index")
        plt.ylabel("Path Length [m]")
        plt.legend()

        plt.savefig("/home/rosjaeger/Desktop/test.png", format='png', bbox_inches='tight', dpi=600)

        plt.show()

    def plot_compare_path_length(self) -> None:
        path_length_list: Dict[str, List[float]] = self.get_path_length()

        # figsize_inches = ((2*88) / 25.4, (2*50) / 25.4)
        figsize_inches = (88 / 25.4, 50 / 25.4)
        plt.figure(figsize=figsize_inches)
        plt.rcParams.update({'font.size': 7.8})
        plt.rcParams['font.family'] = 'serif'
        plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
        # plt.rcParams['font.family'] = 'Times New Roman'

        path_length = next(iter(path_length_list.values()))
        index_list: List[int] = range(len(path_length))

        for key, value in path_length_list.items():
            plt.plot(value, label=key, marker="o", linestyle="-")

        plt.xlim(-3, 103)  # Set x-axis from 2 to 8
        plt.ylim(0, 50)

        plt.xlabel("Index")
        plt.ylabel("Path Length [m]")
        plt.legend()

        plt.savefig("/home/rosjaeger/Desktop/test.png", format='png', bbox_inches='tight', dpi=600)

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

        figsize_inches = (88 / 25.4, 50 / 25.4)
        plt.figure(figsize=figsize_inches)
        plt.rcParams.update({'font.size': 7.8})
        plt.rcParams['font.family'] = 'serif'
        plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']

        plt.bar(index_list, mean_value_list, width=1.0, label="Mean Planning Time", color="#018571")
        plt.bar([i for i in index_list], max_error_list, width=1.0, label="Max Time Difference", color="#204240")
        # plt.plot([mean_error for i in range(len(max_error_list))], color="#FF00FF", linestyle="dashed", label="average max error")
        # plt.plot([mean_planning_time for i in range(len(max_error_list))], color="#30D5C8", linestyle="dashed", label="overall average planning time")


        plt.xlim(-5, 105)  # Set x-axis from 2 to 8
        plt.ylim(0, 90)

        plt.xlabel("Index")
        plt.ylabel("Planning time [ms]")
        plt.legend()
        
        plt.savefig("/home/rosjaeger/Desktop/test.png", format='png', bbox_inches='tight', dpi=600)

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

    def plot_compare_planning_time_bar(self) -> None:
        figsize_inches = (88 / 25.4, 50 / 25.4)
        plt.figure(figsize=figsize_inches)
        plt.rcParams.update({'font.size': 7.8})
        plt.rcParams['font.family'] = 'serif'
        plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']

        planning_time_list: Dict[str, List[float]] = self.get_planning_time()
        index_list: List[int] = range(len(next(iter(planning_time_list.values()))))

        # for key, value in planning_time_list.items():
        #     linestyle: str = None
        #     marker: str = "o"
        #     color:str = None

        #     if "Desktop" in key:
        #         color="red"
        #     elif "Cluster" in key:
        #         color="blue"
        #     elif "Laptop" in key:
        #         color="green"

        #     if "NavFN" in key:
        #         linestyle="dotted"
        #     elif "Smac" in key:
        #         linestyle="dashed"
        #     elif "ThetaStar" in key:
        #         linestyle="dashdot"

            # plt.plot(value, label=key,color=color, marker=marker, linestyle=linestyle)
        bars_A = plt.bar(index_list, planning_time_list["Cluster - Smac"], width=1.0, label="Cluster", color="#018571")
        bars_B = plt.bar(index_list, planning_time_list["Desktop - Smac"], width=1.0, label="Desktop", color="#1e4c49")
        bars_C = plt.bar(index_list, planning_time_list["Laptop - Smac"], width=1.0, label="Laptop", color="#1e2424")

        for bar_A, bar_B, bar_C in zip(bars_A, bars_B, bars_C):
            # Order bars by height (smallest in front)
            order = sorted([bar_A.get_height(), bar_B.get_height(), bar_C.get_height()])
            bar_A.set_zorder(4 - order.index(bar_A.get_height()))  # Smaller bars in front
            bar_B.set_zorder(4 - order.index(bar_B.get_height()))  # Smaller bars in front
            bar_C.set_zorder(4 - order.index(bar_C.get_height()))  # Smaller bars in front

        plt.xlim(-5, 105)  # Set x-axis from 2 to 8
        plt.ylim(0, 1050)

        plt.xlabel("Index")
        plt.ylabel("Planning Time [ms]")
        plt.legend(ncol=3, columnspacing=0.95)

        plt.savefig("/home/rosjaeger/Desktop/test.png", format='png', bbox_inches='tight', dpi=600)

        plt.show()

    def plot_compare_path_length_bar(self) -> None:
        figsize_inches = (88 / 25.4, 50 / 25.4)
        plt.figure(figsize=figsize_inches)
        plt.rcParams.update({'font.size': 7.8})
        plt.rcParams['font.family'] = 'serif'
        plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']

        path_length_list: Dict[str, List[float]] = self.get_path_length()
        index_list: List[int] = range(len(next(iter(path_length_list.values()))))

        # for key, value in planning_time_list.items():
        #     linestyle: str = None
        #     marker: str = "o"
        #     color:str = None

        #     if "Desktop" in key:
        #         color="red"
        #     elif "Cluster" in key:
        #         color="blue"
        #     elif "Laptop" in key:
        #         color="green"

        #     if "NavFN" in key:
        #         linestyle="dotted"
        #     elif "Smac" in key:
        #         linestyle="dashed"
        #     elif "ThetaStar" in key:
        #         linestyle="dashdot"

            # plt.plot(value, label=key,color=color, marker=marker, linestyle=linestyle)
        bars_A = plt.bar(index_list, path_length_list["Cluster - Smac"], width=1.0, label="Cluster", color="#018571")
        bars_B = plt.bar(index_list, path_length_list["Desktop - Smac"], width=1.0, label="Desktop", color="#1e4c49")
        bars_C = plt.bar(index_list, path_length_list["Laptop - Smac"], width=1.0, label="Laptop", color="#1e2424")

        for bar_A, bar_B, bar_C in zip(bars_A, bars_B, bars_C):
            # Order bars by height (smallest in front)
            order = sorted([bar_A.get_height(), bar_B.get_height(), bar_C.get_height()])
            bar_A.set_zorder(4 - order.index(bar_A.get_height()))  # Smaller bars in front
            bar_B.set_zorder(4 - order.index(bar_B.get_height()))  # Smaller bars in front
            bar_C.set_zorder(4 - order.index(bar_C.get_height()))  # Smaller bars in front

        plt.xlim(-5, 105)  # Set x-axis from 2 to 8
        plt.ylim(0, 50)

        plt.xlabel("Index")
        plt.ylabel("Path Length [ms]")
        plt.legend(ncol=3, columnspacing=0.95)

        plt.savefig("/home/rosjaeger/Desktop/test.png", format='png', bbox_inches='tight', dpi=600)

        plt.show()

    def calc_mean_path_length(self, extractor_list: Dict[str, List[RosBagDataExtractor]]) -> List[float]:
        number_of_tests: int = len(next(iter(extractor_list.values())))
        mean_list: List[float] = [0.0 for i in range(number_of_tests)]

        for key, value in extractor_list.items():
            for index, rosbag_extractor in enumerate(value):
                mean_list[index] = round(mean_list[index] + rosbag_extractor.path_length / float(len(extractor_list)), 2)

        # for index, value in enumerate(mean_list):
        #     if value == -1:
        #         mean_list[index] = 0

        return mean_list

    def calc_max_diff_path_length(self, extractor_list: Dict[str, List[RosBagDataExtractor]]) -> List[float]:
        number_of_tests: int = len(next(iter(extractor_list.values())))
        max_diff_list: List[float] = [None for i in range(number_of_tests)]

        mean_value_list: List[float] = self.calc_mean_path_length(extractor_list)

        for key, value in extractor_list.items():
            for index, rosbag_extractor in enumerate(value):
                diff: float = abs(mean_value_list[index] - rosbag_extractor.path_length)
                if max_diff_list[index] is None or \
                    max_diff_list[index] < diff:
                    max_diff_list[index] = round(diff, 2)

        return max_diff_list

    def plot_path_length_with_diff(self) -> None:
        navfn_extractor_list: Dict[str, List[RosBagDataExtractor]] = {k: v for k, v in self._rosbag_data_extractor.items() if "NavFN" in k}
        navfn_path_length_mean_list: List[float] = self.calc_mean_path_length(navfn_extractor_list) 
        navfn_path_length_max_diff_list: List[float] = self.calc_max_diff_path_length(navfn_extractor_list) 
        
        smac_extractor_list: Dict[str, List[RosBagDataExtractor]] = {k: v for k, v in self._rosbag_data_extractor.items() if "Smac" in k}
        smac_path_length_mean_list: List[float] = self.calc_mean_path_length(smac_extractor_list) 
        smac_path_length_max_diff_list: List[float] = self.calc_max_diff_path_length(smac_extractor_list) 
        
        thetastar_extractor_list: Dict[str, List[RosBagDataExtractor]] = {k: v for k, v in self._rosbag_data_extractor.items() if "ThetaStar" in k}
        thetastar_path_length_mean_list: List[float] = self.calc_mean_path_length(thetastar_extractor_list) 
        thetastar_path_length_max_diff_list: List[float] = self.calc_max_diff_path_length(thetastar_extractor_list)

        figsize_inches = (88 / 25.4, 120 / 25.4)
        fig, (ax1, ax2, ax3) = plt.subplots(3, figsize=figsize_inches)
        plt.rcParams['font.family'] = 'serif'
        plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
        plt.rcParams.update({'font.size': 7.8})

        index_list: List[int] = range(len(navfn_path_length_mean_list))

        ax1.bar(index_list, navfn_path_length_mean_list, width=1.0, color='#018571')
        ax2.bar(index_list, smac_path_length_mean_list, width=1.0, color='#186259')
        ax3.bar(index_list, thetastar_path_length_mean_list, width=1.0, color='#204240')

        ax1.errorbar(index_list, navfn_path_length_mean_list, yerr=navfn_path_length_max_diff_list, fmt='', linestyle="None", ecolor='#1e2424', capsize=2, capthick=1, color='#018571', label="NavFN")
        ax2.errorbar(index_list, smac_path_length_mean_list, yerr=smac_path_length_max_diff_list, fmt='', linestyle="None", ecolor='#1e2424', capsize=2, capthick=1, color='#186259', label="Smac")
        ax3.errorbar(index_list, thetastar_path_length_mean_list, yerr=thetastar_path_length_max_diff_list, fmt='', linestyle="None", ecolor='#1e2424', capsize=2, capthick=1, color='#204240', label="Theta Star")

        ax1.set_xlim(-5, 105)
        ax1.set_ylim(0, 55)
        ax2.set_xlim(-5, 105)
        ax2.set_ylim(0, 55)
        ax3.set_xlim(-5, 105)
        ax3.set_ylim(0, 55)

        # # Set titles and labels
        # # plt.title(title)
        # plt.xlabel("Index")
        ax1.set_ylabel("Path Length [m]", fontname=['Times New Roman'], fontsize=7.8)
        ax2.set_ylabel("Path Length [m]", fontname=['Times New Roman'], fontsize=7.8)
        ax3.set_ylabel("Path Length [m]", fontname='Times New Roman', fontsize=7.8)

        ax3.set_xlabel("Index", fontname='Times New Roman', fontsize=7.8)

        font = font_manager.FontProperties(family='Times New Roman', size=7.8)

        ax1.legend(loc="upper right")
        ax2.legend(loc="upper right")
        ax3.legend(loc="upper right")
        plt.subplots_adjust(hspace=0.3)
        plt.savefig("/home/rosjaeger/Desktop/test.png", format='png', bbox_inches='tight', dpi=600)

        # # Show the plot
        fig.tight_layout()
        plt.show()

    def plot_costmap(self) -> None:
        figsize_inches = (88 / 25.4, 50 / 25.4)
        plt.figure(figsize=figsize_inches)
        plt.rcParams.update({'font.size': 7.8})
        plt.rcParams['font.family'] = 'serif'
        plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
        
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
        plt.xlabel("X-axis [m]")
        plt.ylabel("Y-axis [m]")

        x = [x for x in range(0, first_extractor.map.info.width, 100)]
        y = [x for x in range(0, first_extractor.map.info.height, 100)]

        # x_ticks = [x * first_extractor.map.info.resolution for x in range(first_extractor.map.info.width)]
        # y_ticks = [x * first_extractor.map.info.resolution for x in range(first_extractor.map.info.height)]

        
        plt.xticks(x, labels=[round(x_label * first_extractor.map.info.resolution,1) for x_label in x])  
        plt.yticks(y, labels=[round(y_label * first_extractor.map.info.resolution,1) for y_label in y])  

        plt.savefig("/home/rosjaeger/Desktop/test.png", format='png', bbox_inches='tight', dpi=600)

        # Show the plot
        plt.show()

if __name__ == "__main__":
    # dataset_path_list: Dict[str,str] = {"Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_1/dataset_18", 
    #                                     "Cluster - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/Smac/dataset_17",
    #                                     "Cluster - ThetaStar": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/ThetaStar/dataset_19"}

    dataset_path_list: Dict[str,str] = {"Cluster - NavFN - c1": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_1/dataset_22", 
                                        "Cluster - NavFN - c2": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_2/dataset_25",
                                        "Cluster - NavFN - c3": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_3/dataset_26",
                                        "Cluster - NavFN - c4": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_4/dataset_27",
                                        "Cluster - NavFN - c5": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_5/dataset_30"}

    # dataset_path_list: Dict[str,str] = {"Cluster - NavFN - c1": "/home/lurz-match/Desktop/CirpDesign2025/Cluster/NavFN/cycle_1/dataset_22", 
    #                                     "Cluster - NavFN - c2": "/home/lurz-match/Desktop/CirpDesign2025/Cluster/NavFN/cycle_2/dataset_25",
    #                                     "Cluster - NavFN - c3": "/home/lurz-match/Desktop/CirpDesign2025/Cluster/NavFN/cycle_3/dataset_26",
    #                                     "Cluster - NavFN - c4": "/home/lurz-match/Desktop/CirpDesign2025/Cluster/NavFN/cycle_4/dataset_27",
    #                                     "Cluster - NavFN - c5": "/home/lurz-match/Desktop/CirpDesign2025/Cluster/NavFN/cycle_5/dataset_30"}

    # dataset_path_list: Dict[str,str] = {"Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_1/dataset_22",
    #                                     "Cluster - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/Smac/dataset_23",
    #                                     "Cluster - ThetaStar": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/ThetaStar/dataset_24",
    #                                     "Desktop - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/NavFN/dataset_170",
    #                                     "Desktop - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/Smac/dataset_171",
    #                                     "Desktop - ThetaStar": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/ThetaStar/dataset_169",
    #                                     "Laptop - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Laptop/NavFN/dataset_35"}
    
    # dataset_path_list: Dict[str,str] = {"Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_1/dataset_22",
    #                                     "Cluster - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/Smac/dataset_23",
    #                                     "Desktop - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/Smac/dataset_171"}

    # dataset_path_list: Dict[str,str] = {"Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_1/dataset_22",
    #                                     "Desktop - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/NavFN/dataset_170",
    #                                     "Laptop - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Laptop/NavFN/dataset_35"}
    
    # dataset_path_list: Dict[str,str] = {"Cluster - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/Smac/dataset_23",
    #                                     "Desktop - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/Smac/dataset_171",
    #                                     "Laptop - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Laptop/Smac/dataset_36"}

    # dataset_path_list: Dict[str,str] = {"Cluster - ThetaStar": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/ThetaStar/dataset_24",
    #                                     "Desktop - ThetaStar": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/ThetaStar/dataset_169",
    #                                     "Laptop - ThetaStar": "/home/rosjaeger/Desktop/CirpDesign2025/Laptop/ThetaStar/dataset_37"}
    
    # dataset_path_list: Dict[str,str] = {"Cluster - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/NavFN/cycle_1/dataset_22",
    #                                     "Desktop - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/NavFN/dataset_170",
    #                                     "Laptop - NavFN": "/home/rosjaeger/Desktop/CirpDesign2025/Laptop/NavFN/dataset_35",
    #                                     "Cluster - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/Smac/dataset_23",
    #                                     "Desktop - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/Smac/dataset_171",
    #                                     "Laptop - Smac": "/home/rosjaeger/Desktop/CirpDesign2025/Laptop/Smac/dataset_36",
    #                                     "Cluster - ThetaStar": "/home/rosjaeger/Desktop/CirpDesign2025/Cluster/ThetaStar/dataset_24",
    #                                     "Desktop - ThetaStar": "/home/rosjaeger/Desktop/CirpDesign2025/Desktop/ThetaStar/dataset_169",
    #                                     "Laptop - ThetaStar": "/home/rosjaeger/Desktop/CirpDesign2025/Laptop/ThetaStar/dataset_37"}

    # Get all rosbags in the dataset
    # rosbag_names: List[str] = [folder_name for folder_name in os.listdir(path_to_dataset) if os.path.isdir(os.path.join(path_to_dataset, folder_name))]
    # rosbag_names.sort() # Sort list as os.listdir function returns unsorted list
    # absolut_rosbag_path_list: List[str] = [os.path.join(path_to_dataset, rosbag_name) for rosbag_name in rosbag_names]

    data_evaluator: DataEvaluator = DataEvaluator(dataset_path_list)
    data_evaluator.create_data_extractor()
    data_evaluator.calc_path_length()
    data_evaluator.read_planning_time_data()

    # data_evaluator.plot_compare_path_length()
    # data_evaluator.plot_path_length()
    # data_evaluator.plot_compare_planning_time()
    # data_evaluator.plot_compare_planning_time_specific()
    # data_evaluator.plot_compare_planning_time_bar()
    # data_evaluator.plot_compare_path_length_bar()
    # data_evaluator.plot_planning_time_with_diff()
    data_evaluator.plot_planning_time_and_diff()
    # data_evaluator.plot_costmap()

    # data_evaluator.plot_path_length_with_diff()

