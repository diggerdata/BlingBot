export ROS_MASTER_URI=http://natepc.dyn.wpi.edu:11311
export ROS_IP=natepc.dyn.wpi.edu

roslaunch turtlebot_teleop keyboard_teleop.launch

# gnome-terminal -e rviz
gnome-terminal -e roslaunch bling_bot bling.launch
