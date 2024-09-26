import os
import yaml

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    TimerAction
)

from launch.substitutions import (
    PathJoinSubstitution,
    TextSubstitution
)
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



# from utils.pipeline_config import PipelineConfig

class PipelineConfig:
    world_name: str
    world_package: str
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
            self.world_name = config_data["world_name"]
            self.world_package = config_data["world_package"]
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

def generate_launch_description():
    pipeline_config: PipelineConfig = PipelineConfig()
    pipeline_config.import_config()

    # Launch of Gazebo with specified world
    world_file = PathJoinSubstitution([get_package_share_directory(pipeline_config.world_package),
                                       "worlds", 
                                       pipeline_config.world_name])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gpp_pipeline"),
                    "launch",
                    "gazebo_world.launch.py"
                ]
            )
        ),
        launch_arguments={
            "world": world_file,
        }.items(),
    )

    return LaunchDescription(
        [
            gz_sim
        ]
    )

# if __name__ == "__main__":
    

#     launch_file_path: str = get_share_file_path_from_package(package_name="mir_gazebo", file_name="mir_gazebo_launch.py")
#     launch_a_launch_file(launch_file_path = launch_file_path, launch_file_arguments = "")

#     launch_file_path: str = get_share_file_path_from_package(package_name="mir_navigation", file_name="amcl.py")
#     launch_a_launch_file(launch_file_path = launch_file_path, launch_file_arguments = "")