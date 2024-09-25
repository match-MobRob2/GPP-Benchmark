
from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)

from launch.substitutions import (
    PathJoinSubstitution,
    PythonExpression,
    LaunchConfiguration,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # world_file = PathJoinSubstitution([get_package_share_directory("dt_gazebo"), 
    #                                    "worlds", 
    #                                    "match_empty_world.sdf"])
    world_cfg = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", 
        description="Please insert full path to sdf-world here. (Use: PathJoinSubstitution)"
    )

    headless = LaunchConfiguration("headless")
    declare_headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description=("Run Gazebo Ignition in the headless mode"),
    )

    headless_cfg = PythonExpression(
        [
            "'--headless-rendering -s -r' if ",
            headless,
            " else '-r'",
        ]
    )
    gz_args = [headless_cfg, " ", world_cfg]

    use_gpu = LaunchConfiguration("use_gpu")
    declare_use_gpu_arg = DeclareLaunchArgument(
        "use_gpu",
        default_value="True",
        description="Whether GPU acceleration is used",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "True",
        }.items(),
    )

    return LaunchDescription(
        [
            declare_world_arg,
            declare_headless_arg,
            declare_use_gpu_arg,
            # Sets use_sim_time for all nodes started below
            # (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
        ]
    )