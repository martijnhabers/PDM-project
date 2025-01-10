from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Find the package share directory for sjtu_drone_bringup
    bringup_dir = get_package_share_directory('sjtu_drone_bringup')
    gazebo_launch_file = os.path.join(bringup_dir, 'launch', 'sjtu_drone_gazebo.launch.py')

    # Include the sjtu_drone_gazebo_launch.py file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    # Define the waypoint_node
    waypoint_node = Node(
        package='sjtu_drone_control',
        executable='drone_position_control',
        name='drone_position_control',
        output='screen'
    )

    # Define the creator_node
    creator_node = Node(
        package='world_creator',
        executable='creator',
        name='creator',
        output='screen'
    )

    # Return the LaunchDescription containing only the provided nodes
    return LaunchDescription([
        #creator_node,
        gazebo_launch,
        waypoint_node
    ])
