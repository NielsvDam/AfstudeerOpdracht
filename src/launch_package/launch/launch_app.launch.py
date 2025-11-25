from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():  
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_detector',
            default_value='false',
            description='Whether to use the fake object detector or not'
        )
    )
    use_fake_detector = LaunchConfiguration('use_fake_detector')

    machine_state_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('machine_state_control'), 'launch', 'machine_state_control.launch.py')
        )
    )
    hmi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hmi'), 'launch', 'hmi.launch.py')
        )
    )

    object_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('object_detector'), 'launch', 'object_detector.launch.py')
        ),
        launch_arguments={
            'use_fake_detector': use_fake_detector
        }.items()
    )

    # rviz configuration
    rviz_config_file = os.path.join(
        get_package_share_directory('launch_package'), #<<! Altered from launch_package : rviz : rv5as_camera.rviz
        'rviz',
        'abb_irb120_setup.rviz' #<<! Change to abb_irb120 rviz package once made. >>
    )
    # moveit configuration for rviz node (robot model, kinematics, etc.)
    moveit_config = MoveItConfigsBuilder("abb_irb120", package_name="abb_irb120_moveit_config").to_moveit_configs() # Was "rv5as" and "melfa_rv5as_moveit_config".

    # rviz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_prototype",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    launches = [
        machine_state_control_launch,
        object_detector_launch,
        rviz_node,
        hmi_launch,
    ] 

    return LaunchDescription(declared_arguments + launches)