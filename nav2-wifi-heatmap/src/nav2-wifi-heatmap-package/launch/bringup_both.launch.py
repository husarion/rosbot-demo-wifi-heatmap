import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    husarion_nav2 = get_package_share_directory('husarion_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    slam_toolbox_conf = LaunchConfiguration('slam_toolbox_conf',
                            default=husarion_nav2+'/config/slam_toolbox.yaml')
    nav2_conf = LaunchConfiguration('nav2_conf',
                            default=husarion_nav2+'/config/nav2_params.yaml')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([husarion_nav2, '/launch/navigation2.launch.py']),
            launch_arguments = {'nav2_conf' : nav2_conf,
                                'use_sim_time' : use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([husarion_nav2, '/launch/slam_toolbox.launch.py']),
            launch_arguments = {'slam_toolbox_conf' : slam_toolbox_conf,
                                'use_sim_time' : use_sim_time}.items(),
        )
    ])


if __name__ == '__main__':
    generate_launch_description()