import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate the filled_world.world file
    sdf_file = os.path.join(
        get_package_share_directory('sjtu_drone_description'),
        'worlds', 'filled_world.world'
    )

    # Path to the WorldListExport.py script
    world_list_export_script = os.path.join(
        get_package_share_directory('world_creator'),
        'test', 'world_creator', 'WorldListExport.py'
    )


    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', world_list_export_script],
            output='screen'
        ),
        Node(
            package='world_creator',  # Replace with the package name containing the creator executable
            executable='creator',  # Replace with the creator executable name
            name='creator',
            output='screen'
        ),
        Node(
            package='prm_navigation',  # Replace with your package name
            executable='prm_node',  # Replace with your prm_node executable name
            name='prm_node',
            output='screen',
        ),
        Node(
            package='prm_navigation',  # Replace with your package name
            executable='visualization_node',  # Replace with your visualization_node executable name
            name='visualization_node',
            output='screen'
        ),
    ])