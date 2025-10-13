from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_detector',
            default_value='false',
            description='Whether to return fake poses or not'
        )
    )
    use_fake_detector = LaunchConfiguration('use_fake_detector')

    config = os.path.join(
      get_package_share_directory('object_detector'),
      'config',
      'params.yaml'
    )

    object_detector_node = Node(
        package="object_detector",
        executable="object_detector",
        output="screen",
        parameters=[
            config,
            {'use_fake_detector': use_fake_detector}
        ],
        # prefix='gdbserver localhost:3000'
    )

    # rviz configuration
    realsense_json_file = os.path.join(
        get_package_share_directory('object_detector'),
        'config',
        'realsense_tuning2.json'
    )

    realsense_camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        output="screen",
        parameters=[
            {"pointcloud.enable": True},
            {"pointcloud.ordered_pc": False},
            {"depth_module.depth_profile": "1280x720x6"},
            {"depth_module.infra_profile": "1280x720x6"},
            {"enable_color": False},
            {"json_file_path": realsense_json_file}  # Add the path to your .json settings file
        ],
        condition=UnlessCondition(use_fake_detector),
    )

    launches = [
        realsense_camera_node,
        object_detector_node
    ]

    return LaunchDescription(declared_arguments + launches)
