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
            default_value='192.168.125.12',
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
    # Section for the gripper control as arguments.
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_port',
            default_value='80',
            description='The port to which the RWS client connects. Usually doesnt have to be changed from the default (80).'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_nickname',
            default_value='abb_irb120',
            description='Arbitrary user nickname/identifier for the robot controller.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'no_connection_timeout',
            default_value='false',
            description='Enable/disable (false/true) the timeout for getting a RWS connection. Useful if the robot has \
            trouble connecting and might need a unpredictable amount of time to connect.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'polling_rate_io',
            default_value='5.0',
            description='Polling rate of RWS IO system, in frequency (Hz). Raising this will improve timing at the \
            cost of causing trouble for other components, and costing more resources.'
        )
    )

    # Initialize Arguments
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    controller_type = LaunchConfiguration('controller_type')
    moveit_config_package = LaunchConfiguration('moveit_config_package')

    robot_port = LaunchConfiguration('robot_port')
    robot_nickname = LaunchConfiguration('robot_nickname')
    no_connection_timeout = LaunchConfiguration('no_connection_timeout')
    polling_rate_io = LaunchConfiguration('polling_rate_io')

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

    # Launching a RWS client to control IO from/with.
    io_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('abb_bringup'), 'launch', 'abb_rws_client.launch.py')
        ),
        launch_arguments={  
            'robot_ip': robot_ip,
            'robot_port': robot_port,
            'robot_nickname': robot_nickname,
            'no_connection_timeout': no_connection_timeout,
            'polling_rate': polling_rate_io
        }.items()
    )


    launches = [
        abb_driver_launch,
        abb_movit_launch,
        standalone_trajectory_executor_launch,
        io_controller_launch,
    ] 

    return LaunchDescription(declared_arguments + launches)