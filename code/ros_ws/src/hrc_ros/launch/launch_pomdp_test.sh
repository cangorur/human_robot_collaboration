#gnome-terminal -e "roslaunch hrc_ros eval_pomdp_IE.launch" & 
gnome-terminal -e "/home/elia/master_thesis/catkin_ws/src/hrc_industry/code/despot_POMDP_robot/build/examples/pomdpx_models/despot_pomdpx -m /home/elia/master_thesis/catkin_ws/src/hrc_industry/code/models/robot_models/proactive_IE_base.pomdpx"


#xterm -hold -title "robot_agent" -e "roslaunch hrc_ros eval_pomdp_IE.launch" & 
#xterm -hold -title "pomdp_model" -e "/home/elia/master_thesis/catkin_ws/src/hrc_industry/code/despot_POMDP_robot_IE/build/examples/pomdpx_models/despot_pomdpx -m /home/elia/master_thesis/catkin_ws/src/hrc_industry/code/models/robot_models/proactive_IE_base.pomdpx" 
