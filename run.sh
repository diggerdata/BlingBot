export ROS_MASTER_URI=http://192.168.1.121:11311
export ROS_IP=192.168.1.121

gnome-terminal -e roscore

rosrun rviz rviz
