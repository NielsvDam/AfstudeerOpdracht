. install/setup.sh
export LC_NUMERIC=en_US.UTF-8 # see https://github.com/frankaemika/franka_ros2/issues/58

if [ $1 = "debug" ];
then
    ros2 launch launch_package launch_app.launch.py -d -a; # Debug level + all subprocesses.
else 
    ros2 launch launch_package launch_app.launch.py;
fi # Normal.