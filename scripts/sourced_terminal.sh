echo "[INFO] Setting up sourced terminal."

ws="$(cd $(dirname ./); pwd)" # Set current folder as "ws"

if [ -f ${ws}/install/setup.bash ]; then
    . install/setup.bash
    echo "[INFO] Setup successfull! Underlay and overlay both loaded." # Source ROS with both underlay and overlay. (Global and workspace packages)
else
    echo "[WARNING] Could not find setup.bash in install folder, only using the ROS2 underlay instead." && source /opt/ros/humble/setup.bash # Only source ROS undelay. (Only global packages)
fi
export QT_QPA_PLATFORM=xcb && echo "[INFO] QT platform set to xcb to prevent crashes on Wayland." # Due to software updates and them being incompatable with wayland, any Qt rendered GUI's will crash if this setting is not applied to the terminal.