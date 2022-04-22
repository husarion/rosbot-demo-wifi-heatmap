import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    husarion_nav2 = get_package_share_directory('husarion_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_toolbox_conf = LaunchConfiguration('slam_toolbox_conf',
                                default=husarion_nav2+'/config/slam_toolbox.yaml')
 
    slam_toolbox_node = Node(
        	parameters=[
                slam_toolbox_conf
        	],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value=use_sim_time
        ),
        slam_toolbox_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
