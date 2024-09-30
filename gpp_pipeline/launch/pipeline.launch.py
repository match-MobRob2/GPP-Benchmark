import os
import yaml

from launch import LaunchDescription

from launch_ros.actions import Node, SetParameter
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)

from launch.substitutions import (
    PathJoinSubstitution,
    TextSubstitution,
    LaunchConfiguration
)
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition

from nav2_common.launch import RewrittenYaml

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

    print("1")
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', 
                          default_value='false', 
                          description='Use simulation (Gazebo) clock if true')
    map_yaml_file = LaunchConfiguration('map')
    map_yaml_file_arg = DeclareLaunchArgument('map',
                          default_value=os.path.join(get_package_share_directory('gpp_gazebo'), 'maps', 'advanced_maze.yaml'),
                          description='Full path to map yaml file to load')
    autostart = LaunchConfiguration('autostart')
    autostart_arg = DeclareLaunchArgument('autostart', 
                          default_value='true', 
                          description='Automatically startup the nav2 stack')

    lifecycle_nodes = ['map_server']

    params_file = LaunchConfiguration('params_file')
    print("2")
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}
    print("3")
    configured_params = RewrittenYaml(
        source_file=params_file, root_key='', param_rewrites=param_substitutions, convert_types=True
    )
    print("4")
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

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(get_package_share_directory('mir_description'), 'rviz', 'mir_visu_full.rviz'),
        description='Define rviz config file to be used.',
    )

    launch_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        # arguments=['-d', rviz_config_file],
        # parameters=[use_sim_time],
    )

    print("5")
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('gpp_pipeline'), 'config', 'nav_config.yaml'),
        description='Full path to the ROS2 parameters file to use',
    )
    print("6")
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
    )
    print("7")

    nav2_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'autostart': autostart}, {'node_names': lifecycle_nodes}],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            map_yaml_file_arg,
            autostart_arg,
            params_file,
            use_sim_time_arg,
            gz_sim,
            map_server,
            launch_rviz,
            nav2_lifecycle_manager_node
        ]
    )

# if __name__ == "__main__":
    

#     launch_file_path: str = get_share_file_path_from_package(package_name="mir_gazebo", file_name="mir_gazebo_launch.py")
#     launch_a_launch_file(launch_file_path = launch_file_path, launch_file_arguments = "")

#     launch_file_path: str = get_share_file_path_from_package(package_name="mir_navigation", file_name="amcl.py")
#     launch_a_launch_file(launch_file_path = launch_file_path, launch_file_arguments = "")