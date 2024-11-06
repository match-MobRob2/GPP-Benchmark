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

def generate_launch_description():
    world_file = PathJoinSubstitution([get_package_share_directory("gpp_gazebo"),
                                       "worlds", 
                                       "maze.sdf"])

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


    test_node = Node(
        package="gpp_pipeline",
        executable="test_node",
        name="test_node",
        output="screen"
    )

    return LaunchDescription(
        [
            gz_sim,
            test_node
        ]
    )