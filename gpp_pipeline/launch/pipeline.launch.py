import os
import yaml

from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
    EmitEvent
)

from launch.events import Shutdown

from launch.event_handlers import OnProcessStart

from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration
)
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition

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

            self.dataset_folder_path: str = config_data["dataset_folder_path"]
            self.rosbag_naming_convention: str = config_data["rosbag_naming_convention"]

            self.world_name: str = config_data["world_name"]
            self.world_package: str = config_data["world_package"]
            self.map_name: str = config_data["map_name"]
            self.map_package: str = config_data["map_package"]

            self.robot_launch_file: str = config_data["robot_launch_file"]
            self.robot_launch_package: str = config_data["robot_launch_package"]

            # self.number_of_tests: int = config_data["number_of_tests"]

            # self.robot_spawn_position_x: float = config_data["robot_spawn_position_x"]
            # self.robot_spawn_position_y: float = config_data["robot_spawn_position_y"]
            # self.robot_spawn_orientation_yaw: float = config_data["robot_spawn_orientation_yaw"]

            # self.robot_target_position_x: float = config_data["robot_target_position_x"]
            # self.robot_target_position_y: float = config_data["robot_target_position_y"]
            # self.robot_target_orientation_yaw: float = config_data["robot_target_orientation_yaw"]

def generate_launch_description():
    auto_kill = LaunchConfiguration('auto_kill')
    auto_kill_arg = DeclareLaunchArgument('auto_kill',
                          default_value='false',
                          description='Automatically kill all nodes and processes after x seconds')

    pipeline_config: PipelineConfig = PipelineConfig()
    pipeline_config.import_config()

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
    
    # Path where the rosbag is saved.
    # This path is given by the config and the pipeline during execution
    rosbag_path = LaunchConfiguration('rosbag_path')
    rosbag_path_arg = DeclareLaunchArgument('rosbag_path',
                          default_value='/tmp',
                          description='Path of the recoreded rosbag')
    
    # Startposition of the robot
    start_robot_x = LaunchConfiguration('start_robot_x')
    start_robot_x_arg = DeclareLaunchArgument('start_robot_x',
                          default_value='0.0',
                          description='x-value for the start position of the robot')

    start_robot_y = LaunchConfiguration('start_robot_y')
    start_robot_y_arg = DeclareLaunchArgument('start_robot_y',
                          default_value='0.0',
                          description='y-value for the start position of the robot')
    
    start_robot_phi = LaunchConfiguration('start_robot_phi')
    start_robot_phi_arg = DeclareLaunchArgument('start_robot_phi',
                          default_value='0.0',
                          description='phi-value for the start position of the robot')

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
        default_value=os.path.join(get_package_share_directory('gpp_pipeline'), 'rviz', 'standard_config.rviz'),
        description='Define rviz config file to be used.',
    )

    launch_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        arguments=['-d', rviz_config_file],
        parameters=[use_sim_time],
    )
    
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gpp_pipeline"),
                    "launch",
                    "localization.launch.py"
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gpp_pipeline"),
                    "launch",
                    "navigation.launch.py"
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     output="screen" ,
    #     arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"]
    # )

    static_tf = Node(
        package="gpp_pipeline",
        executable="static_frame_publisher_node",
        output="screen",
        parameters=[
            {"start_robot_x": 3.0},
            {"start_robot_y": 3.0},
            {"start_robot_phi": 0.0}
        ]
    )

    ### Data Recording
    # Check if folder is existing
    # counter: int = 0
    # is_folder_existing: bool = True
    # folder_path:str = None
    # while is_folder_existing:
    #     folder_path = os.path.join(pipeline_config.dataset_folder_path + str(counter))
    #     print(folder_path)
    #     is_folder_existing = os.path.isdir(folder_path)
    #     counter = counter + 1

    # if not os.path.isdir(folder_path):
    #     os.makedirs(folder_path)
    
    # index_of_rosbag: int = len(next(os.walk(folder_path))[1])
    # rosbag_path:str = os.path.join(folder_path, pipeline_config.rosbag_naming_convention + str(index_of_rosbag))

    # Old version which doesnt accept param from outside
    # rosbag_record = ExecuteProcess(
    #     cmd=['ros2', 'bag', 'record', '-a', '-o', rosbag_path],
    #     output='screen'
    # )

    rosbag_record = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gpp_pipeline"),
                    "launch",
                    "rosbag_record.launch.py"
                ]
            )
        ),
        launch_arguments={
            "rosbag_path": rosbag_path
        }.items()
    )

    send_new_goal_node = Node(
        package="gpp_pipeline",
        executable="send_new_goal_node",
        name="send_new_goal_node",
        output="screen"
    )
    # start_send_new_goal = RegisterEventHandler(
    #     OnProcessStart(target_action=rosbag_record,
    #                    on_start=[LogInfo(msg="rosbag record started. Sending navigation goal."),
    #                              send_new_goal_node])
    # )

    send_new_goal_delayed = TimerAction(period=5.0, actions=[send_new_goal_node])


    # kill_all_delayed = TimerAction(period=20.0, actions=[EmitEvent(event=Shutdown(reason="PLEASE WORK!"))],)

    create_position_list_node = Node(
        package="gpp_pipeline",
        executable="create_position_list_node",
        name="create_position_list_node",
        output="screen"
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            map_yaml_file_arg,
            autostart_arg,
            rosbag_path_arg,
            use_sim_time_arg,
            declare_rviz_config_arg,
            gz_sim,
            launch_rviz,
            localization,
            navigation,
            static_tf,
            # send_new_goal_node,
            rosbag_record,
            # start_send_new_goal,
            # kill_all_delayed,
            # send_new_goal_delayed,
            # create_position_list_node
        ]
    )