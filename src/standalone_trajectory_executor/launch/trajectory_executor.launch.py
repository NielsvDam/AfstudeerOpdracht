from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():  
    config = os.path.join(
        get_package_share_directory('standalone_trajectory_executor'),
        'config',
        'executor_params.yaml'
    )

    standalone_trajectory_executor_node = Node(
        package="standalone_trajectory_executor",
        executable="standalone_trajectory_executor",
        output="screen",
        parameters=[config]
    )

    return LaunchDescription([
        standalone_trajectory_executor_node,
    ])
