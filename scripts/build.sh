catkin build --workspace /root/catkin_ws
# If setup.bash exists and hasn't been sourced, source it
[[ ":$ROS_PACKAGE_PATH:" != *":/root/catkin_ws"* ]] && [ -f /root/catkin_ws/devel/setup.bash ] && source /root/catkin_ws/devel/setup.bash
