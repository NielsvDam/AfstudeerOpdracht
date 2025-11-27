from traitlets import default
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():  

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Use fake hardware'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.10.11', # <<! ABB - Change IP to match service port host address. >>
            description='Robot IP address'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_type',
            default_value='D',
            description='Controller type'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_package',
            default_value='abb_irb120_moveit_config',
            description='MoveIt configuration package for the robot, e.g. abb_irb1200_5_90_moveit_config'
        )
    )

    # Initialize Arguments
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    controller_type = LaunchConfiguration('controller_type')
    moveit_config_package = LaunchConfiguration('moveit_config_package')

    abb_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('abb_bringup'), 'launch', 'abb_control.launch.py') # Was originally "melfa_bringup" and "rv5as_control_custom.launch.py"
        ),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware,
            'controller_type': controller_type,
            'moveit_config_package' : moveit_config_package,
            'rws_ip': robot_ip
        }.items()
    )

    abb_movit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( 
            os.path.join(get_package_share_directory('abb_bringup'), 'launch', 'abb_moveit.launch.py') # Was "melfa_rv5as_moveit_config" and "rv5as_moveit.launch.py"
        )
    )

    standalone_trajectory_executor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('standalone_trajectory_executor'), 'launch', 'trajectory_executor.launch.py')
        )
    )

    launches = [
        abb_driver_launch, # <<! Change name, looks better. >>
        abb_movit_launch, # <<! Change name, looks better. >>
        standalone_trajectory_executor_launch
    ] 

    return LaunchDescription(declared_arguments + launches)