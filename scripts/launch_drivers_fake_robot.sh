. install/setup.sh
export LC_NUMERIC=en_US.UTF-8 # see https://github.com/frankaemika/franka_ros2/issues/58
export QT_QPA_PLATFORM=xcb # Fix a Qt issue after some updates on Ubuntu.
ros2 launch launch_package launch_drivers.launch.py use_fake_hardware:=true robot_ip:=192.168.125.1