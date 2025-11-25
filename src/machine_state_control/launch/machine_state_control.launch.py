from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():

    config = os.path.join(
      get_package_share_directory('machine_state_control'),
      'config',
      'abb_irb120_params.yaml' # 'rv5as_params.yaml'
    )

    moveit_config = MoveItConfigsBuilder("abb_irb120", package_name="abb_irb120_moveit_config").to_moveit_configs() #<<! ABB Addition Needed >>
    # Original line for MELFA: moveit_config = MoveItConfigsBuilder("rv5as", package_name="melfa_rv5as_moveit_config").to_moveit_configs()
    machine_state_control_node = Node(
        package="machine_state_control",
        executable="machine_state_control",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            config,
            {'use_sim_time': True},  # Add use_sim_time parameter
        ]
    )

    return LaunchDescription([
        machine_state_control_node
    ])