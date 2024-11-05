import os
import yaml

class PipelineConfig:
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
            
            self.rosbag_folder_name: str = config_data["rosbag_folder_name"]
            self.rosbag_naming_convention: str = config_data["rosbag_naming_convention"]

            self.world_name: str = config_data["world_name"]
            self.world_package: str = config_data["world_package"]
            self.map_name: str = config_data["map_name"]
            self.map_package: str = config_data["map_package"]

            self.robot_launch_file: str = config_data["robot_launch_file"]
            self.robot_launch_package: str = config_data["robot_launch_package"]

            self.number_of_tests: int = config_data["number_of_tests"]

            self.robot_spawn_position_x: float = config_data["robot_spawn_position_x"]
            self.robot_spawn_position_y: float = config_data["robot_spawn_position_y"]
            self.robot_spawn_orientation_yaw: float = config_data["robot_spawn_orientation_yaw"]

            self.robot_target_position_x: float = config_data["robot_target_position_x"]
            self.robot_target_position_y: float = config_data["robot_target_position_y"]
            self.robot_target_orientation_yaw: float = config_data["robot_target_orientation_yaw"]