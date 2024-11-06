import os

from signal import SIGINT
import subprocess

from typing import Dict, List
import yaml

if __name__ == "__main__":

    counter: int = 0
    is_folder_existing: bool = True
    folder_path:str = None
    while is_folder_existing:
        folder_path = os.path.join("/home/rosjaeger/rosbag_data/dataset_" + str(counter))
        print(folder_path)
        is_folder_existing = os.path.isdir(folder_path)
        counter = counter + 1

    # Create new dataset folder
    if not os.path.isdir(folder_path):
        os.makedirs(folder_path)
    
    # rosbag_path:str = os.path.join(folder_path, pipeline_config.rosbag_naming_convention + str(index_of_rosbag))

    # Read position data from file
    position_data: Dict[str, Dict[str, Dict[str, float]]]
    with open("/home/rosjaeger/Desktop/position.yaml", 'r', encoding="utf-8") as file:
        position_data = yaml.safe_load(file)

    position_list: List[Dict[str, Dict[str, float]]] = list(position_data.values())

    for experiment_counter in range(len(position_data.items())):
        print("Start run: " + str(experiment_counter))
        
        current_position: Dict[str, Dict[str, float]] = position_list[experiment_counter]
        start_position: Dict[str, float] = current_position["start_position"]
        target_position: Dict[str, float] = current_position["target_position"]

        rosbag_path: str = folder_path + "/rosbag_" + str(experiment_counter)
        launch_process = subprocess.Popen(["ros2", "launch", "gpp_pipeline", "pipeline.launch.py", 
                                           "rosbag_path:=" + rosbag_path,
                                           "start_robot_x:=" + str(start_position["x"]),
                                           "start_robot_y:=" + str(start_position["y"]),
                                           "start_robot_phi:=" + str(start_position["phi"]),
                                           "target_robot_x:=" + str(target_position["x"]),
                                           "target_robot_y:=" + str(target_position["y"]),
                                           "target_robot_phi:=" + str(target_position["phi"])], text=True)
        
        # launch_process = subprocess.Popen(["ros2", "launch", "gpp_pipeline", "pipeline.launch.py", 
        #                                    "rosbag_path:=" + rosbag_path], text=True)

        launch_process.wait() # Wait for launch file to be killed
        # launch_process.send_signal(SIGINT) # Stop launch file. Dont know if still necessary?
        os.system("kill $(ps aux | grep 'ign gazebo gui' | awk '{print $2}')") # Kill Gazebo Ingition explicit
        launch_process.wait(timeout=30)
        print("End run: " + str(experiment_counter))