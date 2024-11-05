import os

from signal import SIGINT
import subprocess

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

    for experiment_counter in range(5):
        print("Start run: " + str(experiment_counter))

        rosbag_path: str = folder_path + "/rosbag_" + str(experiment_counter)

        launch_process = subprocess.Popen(["ros2", "launch", "gpp_pipeline", "pipeline.launch.py", "rosbag_path:=" + rosbag_path], text=True)
        launch_process.wait() # Wait for launch file to be killed
        # launch_process.send_signal(SIGINT) # Stop launch file. Dont know if still necessary?
        os.system("kill $(ps aux | grep 'ign gazebo gui' | awk '{print $2}')") # Kill Gazebo Ingition explicit
        launch_process.wait(timeout=30)
        print("End run: " + str(experiment_counter))