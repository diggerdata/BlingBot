export ROS_MASTER_URI=http://192.168.1.101:11311
export ROS_IP=192.168.1.101

gnome-terminal -e rviz
roslaunch bling_bot bling.launch
