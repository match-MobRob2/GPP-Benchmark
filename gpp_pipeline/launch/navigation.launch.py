import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch.actions import (
    DeclareLaunchArgument
)
from launch.substitutions import (
    LaunchConfiguration
)
from ament_index_python.packages import get_package_share_directory

from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
                          default_value='false',
                          description='Use simulation (Gazebo) clock if true')
    autostart = LaunchConfiguration('autostart')
    autostart_arg = DeclareLaunchArgument('autostart',
                          default_value='true',
                          description='Automatically startup the nav2 stack')
    nav_params_file = LaunchConfiguration('nav_params_file')
    nav_params_file_arg = DeclareLaunchArgument('nav_params_file',
                          default_value=os.path.join(get_package_share_directory('gpp_pipeline'),
                                                     'config',
                                                     'navigation_config.yaml'),
                          description='Full path to the ROS2 parameters file to use')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_nav_to_pose_bt_xml_arg = DeclareLaunchArgument('default_nav_to_pose_bt_xml',
                          default_value=os.path.join(get_package_share_directory('gpp_pipeline'),
                                                     'behavior_trees', 
                                                     'navigate_to_pose.xml'),
                          description='Full path to the behavior tree xml file to use')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    map_subscribe_transient_local_arg = DeclareLaunchArgument('map_subscribe_transient_local',
                          default_value='true',
                          description='Whether to set the map subscriber QoS to transient local')
    

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')] # ('cmd_vel', command_topic)

    # Create our own temporary YAML files that include substitutions
    # Watch out for parameters that don't exist in yaml - will not be substituted of course
    # (default_nav_to_pose_bt_xml)
    # TODO: Needs to be addressed in nav2
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local,
    }

    configured_params = RewrittenYaml(
        source_file=nav_params_file, root_key='', param_rewrites=param_substitutions, convert_types=True
    )

    # Global Path Planner & Costmap
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'debug'],
        remappings=remappings,
    )

    navigation_recovery_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'debug'],
        remappings=remappings,
    )

    navigation_behaviour_tree_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[ParameterFile(configured_params, allow_substs=True), {'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml}],
        arguments=['--ros-args', '--log-level', 'debug'],
        remappings=remappings,
    )

    navigation_lifecylce_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'autostart': autostart}, 
                    {'node_names': ['planner_server', 'bt_navigator', 'recoveries_server']}],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            autostart_arg,
            nav_params_file_arg,
            default_nav_to_pose_bt_xml_arg,
            map_subscribe_transient_local_arg,
            planner_server_node,
            navigation_recovery_server,
            navigation_behaviour_tree_node,
            navigation_lifecylce_manager_node
        ]
    )