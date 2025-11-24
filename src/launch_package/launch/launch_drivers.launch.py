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
            default_value='192.168.10.40', # <<! ABB - Change IP to match service port host address. >>
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

    # Initialize Arguments
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    controller_type = LaunchConfiguration('controller_type')

    melfa_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('abb_bringup'), 'launch', 'abb_control.launch.py') # Was originally "melfa_bringup" and "rv5as_control_custom.launch.py"
        ),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware,
            'controller_type': controller_type,
            'robot_ip': robot_ip
        }.items()
    )

    melfa_movit_launch = IncludeLaunchDescription(
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
        melfa_driver_launch, # <<! Change name, looks better. >>
        melfa_movit_launch, # <<! Change name, looks better. >>
        standalone_trajectory_executor_launch
    ] 

    return LaunchDescription(declared_arguments + launches)