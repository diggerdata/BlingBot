export ROS_MASTER_URI=http://ripnate.dyn.wpi.edu:11311
export ROS_IP=ripnate.dyn.wpi.edu

roslaunch turtlebot_teleop keyboard_teleop.launch

# gnome-terminal -e rviz
gnome-terminal -e roslaunch bling_bot bling.launch
