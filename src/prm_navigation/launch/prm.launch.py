import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription([
        Node(
            package='prm_navigation',  # Replace with your package name
            executable='prm_node',  # Replace with your prm_node executable name
            name='prm_node',
            output='screen',
            parameters=[{"use_sim_time": use_sim_time}]
        ),
        Node(
            package='prm_navigation',  # Replace with your package name
            executable='visualization_node',  # Replace with your visualization_node executable name
            name='visualization_node',
            output='screen',
            parameters=[{"use_sim_time": use_sim_time}]
        ),
    ])
    