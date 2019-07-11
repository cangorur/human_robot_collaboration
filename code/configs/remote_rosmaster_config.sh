# This script is used to setup the remote rosmaster on the Decision PC, if you want to use it with another PC, please change the Ros_Hostname to your local IP address (ifconfig to check)
export ROS_MASTER_URI=http://130.149.232.237:11311 && export ROS_HOSTNAME=130.149.232.26  && echo $ROS_MASTER_URI
