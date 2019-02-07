from websocket import create_connection
import json
import time 

ws = create_connection("ws://localhost:7070")

#repeat ranges 
short_range = list(range(0,3)  )
mid_range =   list(range(0,6) )
long_range =  list(range(0,12) ) 



raw_input("\n\n##################################################\nPlease also start the \n 1) robot_agent (roslaunch hrc_ros hrc_IE.launch)   & \n 2) the pomdp evaluator with logging ( alias = evaluate_robot_model=/home/elia/master_thesis/catkin_ws/src/hrc_industry/code/despot_POMDP_robot_IE/build/examples/pomdpx_models/despot_pomdpx -m /home/elia/master_thesis/catkin_ws/src/hrc_industry/code/models/robot_models/proactive_IE_base.pomdpx)  \n\n           press ENTER to continue ..." )


print "Sending the state request"

for i in short_range:
# *************  step 1 -10 ********* 
# OBS = 9  | standard action IDLE
# state = TaskHuman 	

	obs = "9"
	state = "0"  # TaskHuman
	ws.send(obs + "," + "-1")
	print "Observation is sent : " + obs + "\n"
	ws.send("-1" + "," + state)
	print "Real state is sent  : " + state + "  = TaskHuman \n\n"
	time.sleep(1.0)


for i in short_range:
# *************  step 2 ********* 
# OBS = 4  | standard action GRASPING
# state = HumanDoingOk 	

	obs = "4"
	state = "11"  # HumanDoingOk
	ws.send(obs + "," + "-1")
	print "Observation is sent : " + obs + "\n"
	ws.send("-1" + "," + state)
	print "Real state is sent  : " + state + "  = HumanDoingOk \n\n"
	time.sleep(1.0)


# grasp_distracted  
# *************  step 3 ********* 
# OBS = 3  | distracted grasp 
# state = HumanDoingOk 	

obs = "3"
state = "11"  # HumanDoingOk
ws.send(obs + "," + "-1")
print "Observation is sent : " + obs + "\n"
ws.send("-1" + "," + state)
print "Real state is sent  : " + state + "  = HumanDoingOk \n\n"
time.sleep(1.0)


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
time.sleep(1.0)



# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

 	
for i in long_range:
# grasp_attempt 2   
# *************  step 3 ********* 
# OBS = 2  | standard no action -> going back to where I started 
# state = HumanDoingOk

	obs = "2"
	state = "0"  # HumanDoingOk
	ws.send(obs + "," + "-1")
	print "Observation is sent : " + obs + "\n"
	ws.send("-1" + "," + state)
	print "Real state is sent  : " + state + "  = HumanDoingOk \n\n"
	time.sleep(1.0)


for i in short_range:
# grasping standard
# *************  step 2 ********** 
# OBS = 4  | standard action GRASPING
# state = HumanDoingOk 	

	obs = "4"
	state = "11"  # HumanDoingOk
	ws.send(obs + "," + "-1")
	print "Observation is sent : " + obs + "\n"
	ws.send("-1" + "," + state)
	print "Real state is sent  : " + state + "  = HumanDoingOk \n\n"
	time.sleep(1.0)


# subtask_fail
# *************  step 2 ********** 
# OBS = 13 | subtask_fail
# state = HumanDoingOk 	

obs = "13"
state = "11"  # HumanDoingOk
ws.send(obs + "," + "-1")
print "Observation is sent : " + obs + "\n"
ws.send("-1" + "," + state)
print "Real state is sent  : " + state + "  = HumanDoingOk \n\n"
time.sleep(1.0)



