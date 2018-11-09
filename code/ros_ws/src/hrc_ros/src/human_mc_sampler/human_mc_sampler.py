#!/usr/bin/env python

from hrc_ros.srv import *
from hrc_ros.msg import *
import rospy
import numpy as np
import xml.etree.ElementTree as ET
import os
import std_srvs.srv

# readability purpose: mapping from id to actionname
id_to_actionname = {-1:"unknown", 0:"grasp", 1:"look_around", 2:"idle", 3:"walk_away", 4:"warn_robot"}
# readability purpose: mapping from actionname to id
actionname_to_id = {"unknown":-1, "grasp":0,"look_around":1,"idle":2,"walk_away":3,"warn_robot":4}
# readability purpose: mapping from robot actionname to id
robot_actions = {"unknown":-1, "idle":0, "grasp":1, "cancel_all_actions":2,"point_to_remind":3,"planning_for_grasping":4}
# readability purpose: mapping from id to statename
id_to_statename = {0:		"TaskHuman",
				   1:		"GlobalSuccess",
				   2:		"GlobalFail",
				   3:		"FailedToGrasp",
				   4:		"NoAttention",
				   5:		"Evaluating",
	               6:		"Tired",
	               7:		"Recovery",
	               8:		"RobotInterfered",
	               9:		"WarningTheRobot",
	               10:		"RobotIsWarned",
				   11:		"TaskRobot"}
# readability purpose: mapping from statename to id
statename_to_id = {"TaskHuman" 			:0,
					"GlobalSuccess"		:1,
					"GlobalFail"		:2,
					"FailedToGrasp"		:3,
					"NoAttention"		:4,
					"Evaluating"		:5,
					"Tired"				:6,
					"Recovery"			:7,
					"RobotInterfered"	:8,
					"WarningTheRobot"	:9,
					"RobotIsWarned"		:10,
					"TaskRobot"		:11}

def returnHumanModel():
	"""
	Read human model and apply changes on the Transition functions to create dynamic human behaviours
	Returns:
		If task count is 0 OR use of dynamic transition disabled OR in evaluation mode:
			Return unmodified human POMDPx model based on human type
		Otherwise, modified human POMDPx model
    """
	# TODO: double check the filenaming !
	path = '../../../../../models/human_models/'
	filename = 'modified_human'
	ext = '.POMDPx'
	# NOTE THAT human_type = human_expertise + human_mood
	human_mood = rospy.get_param('/human_mood')
	human_expertise = rospy.get_param('/human_expertise')
	human_type = rospy.get_param('/human_type')
	task_count = rospy.get_param('/task_count')
	interfered_count = rospy.get_param('/robot_interfered_count')
	# warningsCount = rospy.get_param('/warning_count')
	useEvaluator = rospy.get_param('/evaluator_flag')
	useTransitionFunction = rospy.get_param('/dynamic_transition_flag')

	# If task count is 0 OR transition funciton is disabled OR in evaluation, no change on the file
	if (useEvaluator):
		path = path + 'Evaluate/'
		filename = human_type
		return readPOMDPx(path, filename)
	elif (not useTransitionFunction) or task_count == 0:
		filename = human_type
		return readPOMDPx(path, filename)

	# Calculate tiredness, take 40 as max number of tasks (as Can used in testing), set upper bound to 1
	tiredness = min(1, task_count / 40)
	# Calculate collaborativeness, set upper bound to 1
	annoyed = min(1, interfered_count / task_count)
	rospy.loginfo("[MC-SAMPLER] Tiredness = " + str(tiredness) + " Annoyed = " + str(annoyed))

	# Set extreme types and actions which influence
	actionlist_case1 = ['idle', 'warn_robot']
	typelist_case1 = ['collaborative', 'non_collaborative']
	actionlist_case2 = ['look_around', 'idle']
	typelist_case2 = ['tired', 'non_tired']

	# Adjust collaborativeness for basic human types i.e. beginner, expert, distracted
	collaborativeness_tables = modifyPOMDPx(actionlist_case1, typelist_case1, path, annoyed)
	tree = readPOMDPx(path, human_expertise)
	i = 0

	for action in actionlist_case1:
		probtable = readProbTable(tree, action)
		probtable.text = collaborativeness_tables[i]
		i += 1

	modified_file = path + filename + ext
	tree.write(modified_file)
	rospy.loginfo("[MC-SAMPLER] Generated " + modified_file +" with adjusted Collaborativeness!")


	# If human type is beginner or expert, we need to adjust both collaborativeness and tiredness
	if(human_expertise == 'beginner' or human_expertise == 'expert'):
		tiredness_tables = modifyPOMDPx(actionlist_case2, typelist_case2, path, tiredness)
		tree = readPOMDPx(path, human_expertise)
		i = 0

		for action in actionlist_case2:
			probtable = readProbTable(tree, action)
			probtable.text = tiredness_tables[i]
			i += 1

		modified_file = path + filename + ext
		tree.write(modified_file)
		rospy.loginfo("[MC-SAMPLER] Generated " + modified_file +" with adjusted Tiredness!")

	if(os.path.isfile(modified_file)):
		return readPOMDPx(path, filename)


def getNextState(action_and_state):
	"""
	Service to sample a new state after certain action was taken
	Args:
		action_and_state: current action and state as ros service request
	Returns:
		ros service response with new state
	"""
	global id_to_actionname
	global id_to_statename
	global statename_to_id
	global actionname_to_id
	global robot_actions
	global curr_robot_action

	# wait till robot acts. This sets curr_robot_action variable
	_wait_for_robot()

	curr_state = action_and_state.current_human_state
	action_taken = action_and_state.human_action

	rospy.loginfo('[MC-SAMPLER] currstate is %s'%id_to_statename[curr_state])

	human_observable_state = rospy.get_param('/human_observable_state')
	rospy.loginfo('[MC-SAMPLER] Observable State:'+str(human_observable_state))
	rospy.loginfo('[MC-SAMPLER] Observable State:'+str(curr_state))
	# if global fail or success detected before draw call
	if(_is_terminal_state(curr_state, human_observable_state)):
		rospy.loginfo('[MC-SAMPLER] GlobalSuccess or GlobalFail detected')
		new_state = curr_state if(human_observable_state == -1) else human_observable_state
		return DrawNewStateMCResponse(new_state)
	# stay in recovery state
	elif(curr_state == statename_to_id['Recovery'] or action_taken == actionname_to_id['walk_away']):
		rospy.loginfo('[MC-SAMPLER] human is in Recovery')
		new_state = statename_to_id['Recovery']
		return DrawNewStateMCResponse(new_state)
	# transition to robotInterfered state after robot is grasping
	elif(curr_robot_action == robot_actions['grasp']):
		rospy.loginfo('[MC-SAMPLER] robot grasped')
		new_state = statename_to_id['RobotInterfered']
		curr_state = new_state
		# there will be a draw to decide if the robot to be warned or keep it cool
		# return DrawNewStateMCResponse(new_state)
	# grasping is handled in get_grasp_outcome function
	elif(action_taken == actionname_to_id['grasp']):
		# Return without sampling as for grasp a sampling already made
		new_state = curr_state
		return DrawNewStateMCResponse(new_state)
	# react to robot pointing
	elif(curr_robot_action == robot_actions['point_to_remind']):
		rospy.loginfo('[MC-SAMPLER] robot was pointing')
		new_state = statename_to_id['Evaluating']
		return DrawNewStateMCResponse(new_state)
	# transition to RobotIsWarned state after robot canceled all actions or stood idle
	elif(action_taken == actionname_to_id['warn_robot'] and (curr_robot_action == robot_actions['idle'] or curr_robot_action == robot_actions['cancel_all_actions'])):
		rospy.loginfo('[MC-SAMPLER] robot is warned')
		new_state = statename_to_id['RobotIsWarned']
		return DrawNewStateMCResponse(new_state)

	tree = returnHumanModel() # this func. returns the human model with or without transition updates
	root = tree.getroot()

	prob_tables = root.find('StateTransitionFunction')

	table_index = 0
	#iterate over probtables of pomdpx
	for probtable in prob_tables.iter('ProbTable'):
		probtable = np.fromstring(str(probtable.text), sep = ' ')
		probtable = probtable.reshape(int(probtable.shape[0]**0.5), int(probtable.shape[0]**0.5))[:-1,:-1]

		if(table_index == action_taken):
			curr_state_distr = probtable[curr_state]
			num_states = curr_state_distr.shape[0]

			#add the sum of all impossible transitions to the max value of the distr and set the impossible transitions to 0
			curr_state_distr[curr_state_distr == np.max(curr_state_distr)] += np.sum(curr_state_distr[curr_state_distr == np.min(curr_state_distr)])
			curr_state_distr[curr_state_distr == np.min(curr_state_distr)] = 0
			new_state = np.random.choice(num_states, 1, p = curr_state_distr)[0]

			#resampling in some special cases
			while(_resampling_needed(curr_state, new_state, action_taken)):
				rospy.logwarn('%s may not be sampled if action is %s. resampling activated'%(id_to_statename[new_state], id_to_actionname[action_taken]))
				new_state = np.random.choice(num_states, 1, p = curr_state_distr)[0]

			break

		table_index +=1

	# if human reacted to observable state
	# TODO: I dont understand this below !!!
	if(new_state == statename_to_id["WarningTheRobot"] and human_observable_state == statename_to_id["RobotInterfered"]):
		rospy.set_param('/human_observable_state', -1)

	rospy.logwarn('[MC-SAMPLER] new_state is %s'%id_to_statename[new_state])

	return DrawNewStateMCResponse(new_state)

def get_grasping_outcome(action_and_state):
	"""
	Service to sample a new state after grasping action was taken
	Args:
		action_and_state: current action and state as ros service request
	Returns:
		ros service response with new state
	"""
	global id_to_actionname
	global id_to_statename
	global statename_to_id
	global actionname_to_id

	curr_state = action_and_state.current_human_state
	grasping = action_and_state.human_action

	# not needed!
	# if(curr_state != statename_to_id['Evaluating']):
	#	rospy.logwarn('[MC-SAMPLER] grasping may only be done from Evaluating state')
	#	new_state = statename_to_id['Evaluating']
	#	return DrawNewStateMCResponse(new_state)

	rospy.loginfo('[MC-SAMPLER] requesting grasp outcome')
	rospy.loginfo('[MC-SAMPLER] currstate is %s'%id_to_statename[curr_state])

	tree = returnHumanModel()
	root = tree.getroot()

	prob_tables = root.find('StateTransitionFunction')

	table_index = 0
	#iterate over probtables of pomdpx
	for probtable in prob_tables.iter('ProbTable'):
		probtable = np.fromstring(str(probtable.text), sep = ' ')
		probtable = probtable.reshape(int(probtable.shape[0]**0.5), int(probtable.shape[0]**0.5))[:-1,:-1]

		if(table_index == grasping):
			curr_state_distr = probtable[curr_state]
			num_states = curr_state_distr.shape[0]

			#add the sum of all impossible transitions to the max value of the distr and set the impossible transitions to 0
			curr_state_distr[curr_state_distr == np.max(curr_state_distr)] += np.sum(curr_state_distr[curr_state_distr == np.min(curr_state_distr)])
			curr_state_distr[curr_state_distr == np.min(curr_state_distr)] = 0

			new_state = np.random.choice(num_states, 1, p = curr_state_distr)[0]

			#resampling in some special cases
			while(_resampling_needed(curr_state, new_state, grasping)):
				rospy.logwarn('%s may not be sampled if action is %s. resampling activated'%(id_to_statename[new_state], id_to_actionname[grasping]))
				new_state = np.random.choice(num_states, 1, p = curr_state_distr)[0]
			break

		table_index +=1

	rospy.loginfo('[MC-SAMPLER] %s action was taken'%id_to_actionname[grasping])
	rospy.loginfo('[MC-SAMPLER] new_state is %s'%id_to_statename[new_state])

	return DrawNewStateMCResponse(new_state)

def _wait_for_robot():
	"""
	Helperfunction waiting for robot to act.
	"""
	global curr_robot_action
	curr_robot_action = -1
	log_warned = False
	while(curr_robot_action == -1):
		if(not log_warned):
			rospy.logwarn('[MC-SAMPLER] human waiting for robot to act')
			log_warned = True
		curr_robot_action = rospy.get_param('/current_robot_action')

	rospy.set_param('/current_robot_action', -1)

def _is_terminal_state(curr_state, human_observable_state):
	"""
	Helperfunction:
		checks if current state or the observable_state is a terminal state i.e. GlobalSuccess or GlobalFail
	"""
	global statename_to_id
	env_states = [curr_state, human_observable_state]
	global_states = [statename_to_id["GlobalSuccess"], statename_to_id["GlobalFail"]]
	return np.any(np.in1d(global_states, env_states))

def _resampling_needed(curr_state, new_state, action_taken):
	"""
	Helperfunction:
		checks if resampling is needed in some cases. occuring cases are commented below
	"""
	global statename_to_id
	cond1 = (new_state == statename_to_id['GlobalFail'])
	#as robotInterfered is observable, going to robotInterfered from any state other than robotInterfered is avoided
	cond2 = (new_state == statename_to_id['RobotInterfered'] and curr_state != statename_to_id['RobotInterfered'])
	#going to globalsuccess after action is any other than "grasp" is avoided
	cond3 = (new_state == statename_to_id['GlobalSuccess'] and action_taken != actionname_to_id['grasp'])
	#after grasping action the only new states should be GlobalSuccess or FailedToGrasp
	#cond4 = (action_taken == actionname_to_id['grasp'] and (new_state != statename_to_id['GlobalSuccess'] and new_state != statename_to_id['FailedToGrasp']))
	return cond1 or cond2 or cond3 # or cond4

def readPOMDPx(path, type):
	"""
	Function to load corresponding human POMDPx model
	Args:
		path:	path of the human models directory
		type:	type of the human (beginner, expert, distracted)
	Returns:
		Top element of the POMDPx model
    """
	pomdpx_file = path + type
	#print('loading... %s' %pomdpx_file)
	return ET.parse(pomdpx_file)

def readProbTable(tree, action):
	"""
	This function parses and reads probability tables
	Args:
		tree:	top element of the POMDPx model
		action:	action of the human
	Returns:
		Probability table corresponding to an action
    """
	root = tree.getroot()
	transition_fct = root.find('StateTransitionFunction')
	index = 0

	for probtable in transition_fct.iter('ProbTable'):
		if(index == actionname_to_id[action]):
			return probtable

		index += 1

def calcDiff(P, factor):
	"""
	This function calculates a new table by subtracting probability tables from extreme types
	Also multiplies the resulting table with factor and then normalizes
	Args:
		P:	list of probability tables of extreme types
		factor:	collaborativeness or tiredness parameter which needs to be adjusted
	Returns:
		Newly calculated and normalized probability table
    """
	# P(t) = P1 + t*(P2-P1)
	d = np.absolute(np.subtract(P[1], P[0]))
	T = np.add(P[0], (factor * d))
	# Normalize
	T = T / T.sum(axis=1, keepdims=1)

	return T

def convertToString(T):
	"""
	This function converts probability table from numpy array to string type
	Args:
		T:	probability table as numpy array
	Returns:
		Probability table as string
    """
	t = '    '.join(map(str, T.flatten()))
	i = 0
	table_str = ""

	for s in t.split():
		if (i % 12 == 11):
			table_str += "   " + s + "\n"
		elif (i == 0):
			table_str = s
		else:
			table_str += "   " + s
		i += 1

	return table_str

def modifyPOMDPx(action_list, type_list, path, factor):
	"""
	Function to modify probability tables based on human action and factor
	Args:
		action_list:	list of human actions
		type_list:	list of human types
		path:	path of the human models directory
		factor:	collaborativeness or tiredness parameter which needs to be adjusted
	Returns:
		Modified probability tables list
    """
	P = []
	modified_tables = []

	for action in action_list:
		i = 0
		for type in type_list:
			tree = readPOMDPx(path, type)
			probtable = readProbTable(tree, action)
			table = np.fromstring(str(probtable.text), sep = ' ')
			table = table.reshape(int(table.shape[0]**0.5), int(table.shape[0]**0.5))
			P.append(table)
			i += 1

		T = calcDiff(P, factor)
		table_str = convertToString(T)
		modified_tables.append(table_str)
		P.clear()

	return modified_tables

if __name__ == "__main__":
	rospy.init_node('human_mc_sampler')
	rospy.Service('/human_mc_sampler/human_mc_sampler', DrawNewStateMC, getNextState)
	rospy.Service('/human_mc_sampler/grasping_outcome', DrawNewStateMC, get_grasping_outcome)

	rospy.loginfo("human sampler ready!")
	rospy.spin()
