from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_publisher_package',
            namespace='waypoint_publisher',
            executable='node',
            name='waypoint_publisher'
        ),
        Node(
            package='rssi_heatmap_generator_package',
            namespace='heatmap_generator',
            executable='node',
            name='heatmap_generator'
        )
    ])