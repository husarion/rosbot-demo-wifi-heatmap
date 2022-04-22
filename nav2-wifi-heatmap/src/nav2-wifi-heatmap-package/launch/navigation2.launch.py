import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    husarion_nav2 = get_package_share_directory('husarion_nav2')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav2_conf = LaunchConfiguration('nav2_conf',
                    default=husarion_nav2+'/config/nav2_params.yaml')

    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params')
    default_bt_xml_filename = LaunchConfiguration(
        'default_bt_xml_filename')

    remappings = []

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
    }

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params',
            default_value=[nav2_conf],
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),


        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

    ])


if __name__ == '__main__':
    generate_launch_description()
