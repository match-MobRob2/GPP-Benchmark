import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration
)
from ament_index_python.packages import get_package_share_directory

from nav2_common.launch import RewrittenYaml

def generate_launch_description():
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

    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('gpp_pipeline'), 'config', 'localization_config.yaml'),
        description='Full path to the ROS2 parameters file to use')

    lifecycle_nodes = ['map_server']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}

    configured_params_map = RewrittenYaml(
        source_file=params_file, root_key='', param_rewrites=param_substitutions, convert_types=True
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params_map],
        remappings=remappings,
    )

    map_lifecycle_manager_node = Node(
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
            params_file_arg,
            map_server_node,
            map_lifecycle_manager_node
        ]
    )