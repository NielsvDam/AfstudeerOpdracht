from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="melfa_rv5as_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )

    # Initialize Arguments
    start_rviz = LaunchConfiguration('start_rviz')
    moveit_config_package = LaunchConfiguration("moveit_config_package")

    # Initialize Moveit Configuration
    moveit_config = (
        MoveItConfigsBuilder("rv5as", package_name="melfa_rv5as_moveit_config")
        .robot_description(
            file_path="config/rv5as.urdf.xacro",
        )
        .robot_description_semantic(file_path="config/rv5as.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            # pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"] # Add "stomp" if moveit2 humble branch adds stomp feature
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
    
    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_capabilities],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "rv5as_moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(start_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Get parameters for the Servo node
    servo_params = (
        ParameterBuilder("melfa_rv5as_moveit_config")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/rv5as_servo.yaml",
        )
        .to_dict()
    )

    # Servo node for realtime control
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics
        ],
        # ros_arguments= [{'use_intra_process_comms' : True}],
        output="screen",
    )

    nodes = [
        move_group_node, 
        # rviz_node, 
        servo_node
    ]

    return LaunchDescription(declared_arguments + nodes)
