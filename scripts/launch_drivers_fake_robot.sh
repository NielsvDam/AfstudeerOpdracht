. install/setup.sh
export QT_QPA_PLATFORM=xcb && echo "[INFO] QT platform set to xcb to prevent crashes on Wayland." # Due to software updates and them being incompatable with wayland, any Qt rendered GUI's will crash if this setting is not applied to the terminal.
export LC_NUMERIC=en_US.UTF-8 && echo "[INFO] LC_NUMERIC is set to UTF-8. to precent possible ROS2 issue." # Taken from launch_app & lauch_drivers, https://github.com/frankaemika/franka_ros2/issues/58
ros2 launch launch_package launch_drivers.launch.py use_fake_hardware:=true robot_ip:=192.168.125.12