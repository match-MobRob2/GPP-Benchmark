import os
import yaml

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