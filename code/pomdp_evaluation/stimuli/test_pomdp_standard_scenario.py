from websocket import create_connection
import json
import time 

ws = create_connection("ws://localhost:7070")

#repeat ranges 
short_range = list(range(0,5)  )
mid_range =   list(range(0,10) )
long_range =  list(range(0,20) ) 



raw_input("\n\n##################################################\nPlease also start the \n 1) robot_agent (roslaunch hrc_ros hrc_IE.launch)   & \n 2) the pomdp evaluator with logging ( alias = evaluate_robot_model=/home/elia/master_thesis/catkin_ws/src/hrc_industry/code/despot_POMDP_robot_IE/build/examples/pomdpx_models/despot_pomdpx -m /home/elia/master_thesis/catkin_ws/src/hrc_industry/code/models/robot_models/proactive_IE_base.pomdpx)  \n\n           press ENTER to continue ..." )


print "Sending the state request"

for i in long_range:
# *************  step 1 -10 ********* 
# OBS = 9  | standard action IDLE
# state = TaskHuman 	

	obs = "9"
	state = "0"  # TaskHuman
	ws.send(obs + "," + "-1")
	print "Observation is sent : " + obs + "\n"
	ws.send("-1" + "," + state)
	print "Real state is sent  : " + state + "  = TaskHuman \n\n"
	time.sleep(0.5)


for i in mid_range:
# *************  step 2 ********* 
# OBS = 4  | standard action GRASPING
# state = HumanDoingOk 	

	obs = "4"
	state = "11"  # HumanDoingOk
	ws.send(obs + "," + "-1")
	print "Observation is sent : " + obs + "\n"
	ws.send("-1" + "," + state)
	print "Real state is sent  : " + state + "  = HumanDoingOk \n\n"
	time.sleep(0.5)


# Subtask_success  
# *************  step 3 ********* 
# OBS = 12  | subtask_success 
# state = HumanDoingOk 	

obs = "12"
state = "11"  # HumanDoingOk
ws.send(obs + "," + "-1")
print "Observation is sent : " + obs + "\n"
ws.send("-1" + "," + state)
print "Real state is sent  : " + state + "  = HumanDoingOk \n\n"
time.sleep(
0.5)
