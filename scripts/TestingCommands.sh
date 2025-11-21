# A list of terminal commands to run with the ROS2 driver to test it.

# Publish a JointTrajectory with 1 point to test EGM functionality.
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header: 
  stamp: 
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
points: 
  - 
    positions: [2.5, -1.17, 0.0, -1.57, 0.0, 0.0]
    velocities: []
    accelerations: []
    effort: []
    time_from_start: 
      sec: 5
      nanosec: 0"

