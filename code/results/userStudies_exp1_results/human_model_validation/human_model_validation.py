#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 24 23:45:29 2020

@author: cangorur
"""
import shelve
import sklearn.metrics as mtr
from scipy.io import loadmat, savemat
import pandas as pd
import os
import numpy as np
import mdptoolbox
import xml.etree.ElementTree as ET

user_obs = []
simu_obs = []
user_taskID_arr = []
simu_taskID_arr = []
simu_subtaskID_arr = []
list_of_models = []
model_ids = []
action_list = ['graspAttempt', 'lookAround', 'idle', 'walkAway', 'warnRobot']
state_list = ['TaskHuman',
              'GlobalSuccess',
              'GlobalFail',
              'FailedToGrasp',
              'NoAttention',
              'Evaluating',
              'Tired',
              'Recovery',
              'RobotInterfered',
              'WarningTheRobot',
              'RobotIsWarned',
              'TaskRobot']
actionname_to_id = {"unknown":-1, "graspAttempt":0,"lookAround":1,"idle":2,"walkAway":3,"warnRobot":4}
statename_to_id = {        "TaskHuman" 		:0,
					"GlobalSuccess"		:1,
					"GlobalFail"		:2,
					"FailedToGrasp"		:3,
					"NoAttention"		:4,
					"Evaluating"		:5,
					"Tired"			:6,
					"Recovery"			:7,
					"RobotInterfered"	:8,
					"WarningTheRobot"	:9,
					"RobotIsWarned"		:10,
					"TaskRobot"		:11}

def readObs():
    # Reading real human data collected from the participants
    global user_obs, user_taskID_arr, simu_obs, simu_taskID_arr, simu_subtaskID_arr
    
    dir_path = os.path.dirname(os.path.realpath(__file__))
    df = pd.read_excel(dir_path + '/../../userStudies_exp2_results/tests/analysis_objective.xlsx', sheet_name='human_observables', sep='\s*,\s*')
    part_id = 4
    total_subjects = 14
    for i in range(total_subjects):
        part_temp_obs = df['Obs_' + str(part_id)].values.tolist()
        part_temp_obs = [x for x in part_temp_obs if str(x) != 'nan']
        [int(i) for i in part_temp_obs]
        user_obs.append(part_temp_obs)
        part_tasks = df['task_id_' + str(part_id)].values.tolist()
        part_tasks = [x for x in part_tasks if str(x) != 'nan']
        [int(i) for i in part_tasks]
        user_taskID_arr.append(part_tasks)
        part_id += 1
        
     # The mappings are necessary for real observations as a stood idle observable always high after a failure or success in grasp is issued. In simulation it is not the case but they reflect the same scenario    
    for user_id in range(len(user_obs)):
        obs = 0        
        while(1):
            real_length = len(user_obs[user_id])
            if obs == real_length - 1: # we reach the end, terminate
                break
            for obs in range(len(user_obs[user_id])):
                observation_number = user_obs[user_id][obs]
                if (observation_number == 41 or observation_number == 43 or observation_number == 40 or observation_number == 24 or observation_number == 11 or observation_number == 10):
                    observation_number = 9
                elif (observation_number == 25 or observation_number == 49 or observation_number == 21 or observation_number == 57 or observation_number == 16 or observation_number == 19 or observation_number == 18):
                    observation_number = 17
                elif (observation_number == 7 or observation_number == 37):
                    observation_number = 5
                elif (observation_number == 34 or observation_number == 35 or observation_number == 32 or observation_number == 1 or observation_number == 2):
                    observation_number = 33
                # if a new subtask starts with a grasp (success or failure) or a warning, i.e., other than 33, add a 33 (idle) OR if a new task starts with 9 or 5 add a 33 (idle) before regardless!
                if (obs != 0 and (observation_number != 33) and ((user_obs[user_id][obs-1] != 33) or (user_taskID_arr[user_id][obs] != user_taskID_arr[user_id][obs-1]))): 
                    if (observation_number == 5 or observation_number == 9):                    
                        user_obs[user_id] = np.insert(user_obs[user_id], obs, 33)
                        user_taskID_arr[user_id] = np.insert(user_taskID_arr[user_id], obs, user_taskID_arr[user_id][obs])
                        user_obs[user_id][obs+1] = observation_number # add the real value as the next observation as the index has changed now
                    if (observation_number == 17):
                        user_obs[user_id] = np.insert(user_obs[user_id], obs, 33)
                        user_obs[user_id] = np.insert(user_obs[user_id], obs, 33)
                        user_taskID_arr[user_id] = np.insert(user_taskID_arr[user_id], obs, user_taskID_arr[user_id][obs])
                        user_taskID_arr[user_id] = np.insert(user_taskID_arr[user_id], obs, user_taskID_arr[user_id][obs])
                        user_obs[user_id][obs+2] = observation_number # add the real value as the next observation as the index has changed now
                else:
                    user_obs[user_id][obs] = observation_number
                
    # Reading simulated human data collected from the simulation runs.
    df = pd.read_excel(dir_path + '/human_observables_simulation.xlsx', sheet_name='Sheet1', sep='\s*,\s*')
    simu_id = 1
    simu_subjects = 8
    for i in range(simu_subjects):
        simu_temp_obs = df['Obs_' + str(simu_id)].values.tolist()
        simu_temp_obs = [x for x in simu_temp_obs if str(x) != 'nan']
        [int(i) for i in simu_temp_obs]
        simu_obs.append(simu_temp_obs)
        simu_tasks = df['task_id_' + str(simu_id)].values.tolist()
        simu_tasks = [x for x in simu_tasks if str(x) != 'nan']
        [int(i) for i in simu_tasks]
        simu_taskID_arr.append(simu_tasks)
        simu_subtasks = df['subtask_id_' + str(simu_id)].values.tolist()
        simu_subtasks = [x for x in simu_subtasks if str(x) != 'nan']
        [int(i) for i in simu_subtasks]
        simu_subtaskID_arr.append(simu_subtasks)
        simu_id += 1
        
     # The mappings are necessary for real observations as a stood idle observable always high after a failure or success in grasp is issued. In simulation it is not the case but they reflect the same scenario    
    for simu_id in range(len(simu_obs)):
        obs = 0
        while(1):
            real_length = len(simu_obs[simu_id])
            if obs == real_length - 1: # we reach the end, terminate
                break
            temp_obs = np.array([])
            temp_task = np.array([])
            temp_subtask = np.array([])
            grasp_complete = False
            grasp_state = ""
            idle_ctr = 0
            for obs in range(len(simu_obs[simu_id])):
                observation_number = simu_obs[simu_id][obs]
                if (observation_number == 41 or observation_number == 43 or observation_number == 40 or observation_number == 24 or observation_number == 11 or observation_number == 8 or observation_number == 10):
                    observation_number = 9
                elif (observation_number == 25 or observation_number == 49 or observation_number == 21 or observation_number == 57 or observation_number == 16 or observation_number == 19 or observation_number == 18 or observation_number == 51):
                    observation_number = 17
                elif (observation_number == 7 or observation_number == 37):
                    observation_number = 5
                elif (observation_number == 34 or observation_number == 35 or observation_number == 32 or observation_number == 0 or observation_number == 1 or observation_number == 2 or observation_number == 3):
                    observation_number = 33
               
                if grasp_complete: # grasp_complete states a grasp has already been issued in this subtask. 
                    if observation_number == 5:
                        grasp_state = "success"
                        observation_number = 33
                    elif observation_number == 9:
                        grasp_state = "failure"
                        observation_number = 33
                    idle_ctr += 1
                elif grasp_complete == False and (simu_subtaskID_arr[simu_id][obs] == simu_subtaskID_arr[simu_id][obs-1]) and (observation_number == 5 or observation_number == 9):
                    grasp_complete = True
                    idle_ctr += 1
                    if observation_number == 5:
                        grasp_state = "success"
                    elif observation_number == 9:
                        grasp_state = "failure"
                    #observation_number = 33
                
                 # Once a grasp is detected, there cannot be a new one coming in a subtask. We remove all of them but keep the latest one by setting this flag if they are in the same subtask.
                if obs!=0 and simu_subtaskID_arr[simu_id][obs] != simu_subtaskID_arr[simu_id][obs-1]:
                    grasp_complete = False
                    grasp_state = ""
                    idle_ctr = 0
                elif (obs != len(simu_obs[simu_id]) -1) and (simu_subtaskID_arr[simu_id][obs] != simu_subtaskID_arr[simu_id][obs+1]):
#                    if grasp_state == "success":
#                        observation_number = 5
#                    if grasp_state == "failure":
#                        observation_number = 9
                    grasp_complete = False
                
                # if a new subtask starts with a grasp (success or failure) or a warning, i.e., other than 33, add a 33 (idle) OR if a new task starts with 9 or 5 add a 33 (idle) before regardless!
                if (obs != 0 and (observation_number != 33) and ((simu_taskID_arr[simu_id][obs] != simu_taskID_arr[simu_id][obs-1]) 
                    or (simu_subtaskID_arr[simu_id][obs] != simu_subtaskID_arr[simu_id][obs-1]))): 
                    if (observation_number == 5 or observation_number == 9):                    
                        temp_obs = np.append(temp_obs, 33)                        
                        temp_task = np.append(temp_task, simu_taskID_arr[simu_id][obs])
                        temp_subtask = np.append(temp_subtask, simu_subtaskID_arr[simu_id][obs])

                    if (observation_number == 17): # add 33 two times until a warning
                        temp_obs = np.append(temp_obs, 33)                        
                        temp_task = np.append(temp_task, simu_taskID_arr[simu_id][obs])
                        temp_subtask = np.append(temp_subtask, simu_subtaskID_arr[simu_id][obs])
                        temp_obs = np.append(temp_obs, 33)                        
                        temp_task = np.append(temp_task, simu_taskID_arr[simu_id][obs])
                        temp_subtask = np.append(temp_subtask, simu_subtaskID_arr[simu_id][obs])                       
                    
                if grasp_complete and idle_ctr > 1:
                    continue 
                
                temp_obs = np.append(temp_obs, observation_number)
                temp_task = np.append(temp_task, simu_taskID_arr[simu_id][obs])
                temp_subtask = np.append(temp_subtask, simu_subtaskID_arr[simu_id][obs])
            
            simu_obs[simu_id] = temp_obs
            simu_taskID_arr[simu_id] = temp_task
            simu_subtaskID_arr[simu_id] = temp_subtask

    my_shelf = shelve.open("user_simu_obs.out",'n') # 'n' for new    
    my_shelf["user_obs"] = user_obs
    my_shelf["simu_obs"] = simu_obs
    my_shelf["user_taskID_arr"] = user_taskID_arr
    my_shelf["simu_taskID_arr"] = simu_taskID_arr
    my_shelf["simu_subtaskID_arr"] = simu_subtaskID_arr
    my_shelf.close()
    #savemat("updated_user_simu_obs.mat",{'user_obs':np.array(user_obs), 'simu_obs':np.array(simu_obs),
     #                                  'user_taskID_arr':np.array(user_taskID_arr),'simu_taskID_arr':np.array(simu_taskID_arr)})
    
    return user_obs, user_taskID_arr, simu_obs, simu_taskID_arr, simu_subtaskID_arr
    
#def KLConvergenceAnalysis_perTask(user_obs, user_taskID_arr, simu_obs, simu_taskID_arr):
    
def removeRepeatingObs(old_obs):
    
    new_obs = []
    for user_id in range(len(old_obs)):
        curr_user = old_obs[user_id]
        new_obs_arr = []
        #prev_obs = curr_user[0]
        #new_obs_arr.append[prev_obs]
        for i in range(len(curr_user)):
            if i == 0:
                prev_obs = curr_user[i]
                new_obs_arr.append(prev_obs)
                continue
            curr_obs = curr_user[i]
            if prev_obs != curr_obs:
                new_obs_arr.append(curr_obs)
                prev_obs = curr_obs
        new_obs.append(new_obs_arr)
    return new_obs
                
    
def KLConvergenceAnalysis(user_obs, user_taskID_arr, simu_obs, simu_taskID_arr):
    
    score_arr_perSimu = []
    all_score_perSimu_max = []
    all_score_perSimu = []
    final_scores = []
    final_scores_max = []
    #user_obs = user_obs[0] # TODO: for initial tests, only one user and one simulated human
    #simu_obs = simu_obs[0]
        
    # searching for a best match for a user obs array in all of the simulated obs (sliding window). It should be within a task?
    for user_id in range(len(user_obs)):
        curr_user = user_obs[user_id]
        curr_user_len = len(curr_user)
        all_score_perSimu_max = []
        all_score_perSimu = []
        for simu_id in range(len(simu_obs)):
            curr_simu = simu_obs[simu_id]
            curr_simu_len = len(curr_simu)
            score_arr_perSimu = []
            for i in range(curr_simu_len - curr_user_len + 1):
                #score_arr.append(mtr.mutual_info_score(curr_simu[i:i+curr_user_len],curr_user))
                score_arr_perSimu.append(mtr.mutual_info_score(curr_simu[i:i+curr_user_len], curr_user))
            all_score_perSimu_max.append(max(score_arr_perSimu))
            all_score_perSimu.append(score_arr_perSimu)
        final_scores_max.append(all_score_perSimu_max)
        final_scores.append(all_score_perSimu)
    return final_scores, final_scores_max
    
def MDPTupleGeneration():
    
    global list_of_models
    
    P_arr = []
    R_arr = []
    dis_arr = []
    init_belief_arr = []
    policy_arr = []
    
    readHumanModel() # reads human models and save the prob trees in "list_of_models"
    
    for model in list_of_models:
        
        P, R, discount, init_belief = readProbTables(model)
        mdptoolbox.util.check(P, R) 
        vi = mdptoolbox.mdp.ValueIteration(P, R, discount)
        vi.run()
        policy = np.array(vi.policy)
        
        P_arr.append(P)
        R_arr.append(R)
        dis_arr.append(discount)
        init_belief_arr.append(init_belief)
        policy_arr.append(policy)
    #print(policy)
    #print(vi.V)
    return P_arr, R_arr, dis_arr, init_belief_arr, policy_arr
    #P, R = mdptoolbox.example.forest()
    #fh = mdptoolbox.mdp.FiniteHorizon(P, R, 0.9, 3)
    #fh.run()
    #print(fh.V)
    #print(fh.policy)
    #print(P)

def readHumanModel():
    """
    	Read human model and apply changes on the Transition functions to create dynamic human behaviours
    	Returns:
    		If task count is 0 OR use of dynamic transition disabled OR in evaluation mode:
    			Return unmodified human POMDPx model based on human type
           Otherwise, modified human POMDPx model
   """
    global list_of_models, model_ids
    
    path = '../../../models/human_models/Evaluate/'
    expertise = ["expert", "beginner"]
    stamina = ["nontired", "tired"]
    col = ["collaborative", "noncollaborative"]
    for expert in expertise:
        for energy in stamina:
            for pref in col:
                filename = expert + "_" + energy + "_" + pref + ".POMDPx"    
                pomdpx_file = path + filename
                #print('loading... %s' %pomdpx_file)
                tree = ET.parse(pomdpx_file)
                list_of_models.append(tree)
                model_ids.append(filename)
    
    
def readProbTables(tree):
    """
    This function parses and reads probability tables
    Args:
        tree:	top element of the POMDPx model
        action:	action of the human
    Returns:
        Transition and reward prob matrices in numpy array formats: T=(A,S,S) , R=(S, A)
    """
    global action_list, state_list
    
    root = tree.getroot()
    transition_fct = root.find('StateTransitionFunction')
    index = 0
    trans_probs = []
    
    reward_array = np.zeros((len(state_list), len(action_list)))
    
    for action in action_list:
        for probtable in transition_fct.iter('ProbTable'):
            if(index == actionname_to_id[action]):
                table = np.fromstring(str(probtable.text), sep = ' ')
                table = table.reshape(int(table.shape[0]**0.5), int(table.shape[0]**0.5))
                trans_probs.append(table)
                #print("reshaped:  ", table)
                break
            index += 1
        index = 0
    
    reward_entries = root.find('RewardFunction')
    index=0
    for entry in reward_entries.iter('Entry'):
        reward_vector = entry.find('Instance').text
        action, state = reward_vector.split(' ')
        value = float(entry.find('ValueTable').text)
        if action == '*':
            action = []
            for a in action_list:
                action.append(actionname_to_id[a])
        else:
            action = actionname_to_id[action]
        if state == '*':
            state = []
            for s in state_list:
                state.append(statename_to_id[s])
        else:
            state = statename_to_id[state]

        reward_array[state, action] = value
        
    discount = float(root.find('Discount').text)
    
    initial_belief = root.find('InitialStateBelief')
    for entr in initial_belief.iter('Entry'):
        txt = str(entr.find('ProbTable').text)
    init_belief = np.fromstring(txt, sep = ' ')
    
    return trans_probs, reward_array, discount, init_belief
    
def MDPmodelLikelihood(simu_flag):
    
    global model_ids, user_obs, user_taskID_arr, simu_obs, simu_taskID_arr, simu_subtaskID_arr
    
    # KL Convergence Analysis    
    
    #dataset1=loadmat("kl_convergence_slidingWindow_fullData_v2.mat")
    #arr = dataset1['final_kl_array_max']
    #dataset1['final_kl_array_max'] = np.array(dataset1)
    #savemat("kl_convergence_slidingWindow_fullData_v2.mat",dataset1)
    
    #final_kl_array_max = []
    #final_kl_array = []
    #user_obs = removeRepeatingObs(user_obs)
    #simu_obs = removeRepeatingObs(simu_obs)
    #final_kl_array, final_kl_array_max = KLConvergenceAnalysis(user_obs, user_taskID_arr, simu_obs, simu_taskID_arr)
    #final_kl_array_max = np.array(final_kl_array_max)
    #print(final_kl_array_max)
    
    # When this flag is true, we check the likelihood of simulated human observations generated by the simulated human models
    if (simu_flag):
        human_obs = simu_obs
        human_taskID_arr = simu_taskID_arr
        human_subtaskID_arr = simu_subtaskID_arr
    else:
        human_obs = user_obs
        human_taskID_arr = user_taskID_arr
    
    T_arr, R_arr, discount_arr, init_belief_arr, policy_arr = MDPTupleGeneration()
    
    if not simu_flag:
        likelihood_arrParticipants = []
    else:
        likelihood_arrParticipants = np.zeros((len(model_ids), len(human_obs)))
    
    for user_id in range(len(human_obs)): # range(len(human_obs))
        #user_id = 6
        curr_human_obs = np.array(human_obs[user_id])
        curr_taskID_arr = np.array(human_taskID_arr[user_id])
        if simu_flag:        
            curr_subtaskID_arr = np.array(human_subtaskID_arr[user_id])
        if not (simu_flag):
            total_task_amount = 6
            likelihood_arrUser = np.zeros((len(model_ids), total_task_amount))
        else:
            likelihood_arrUser = 0.0 # in simulation, as the data are generated from the models directly, we do not need to separate the tasks. We average them all.

        # For every simulated human model, we calculate the likelihood of a participant's observations generated.
        for model_id in range(len(model_ids)): # range(len(model_ids))
            #model_id = 6
            print("=== Calculating the likelihood of user:", user_id, " with the model: ", model_ids[model_id], "===")
            T = T_arr[model_id]
            init_belief = init_belief_arr[model_id]
            policy = policy_arr[model_id]
            
            print("POLICY: ", policy)
            curr_belief = init_belief
            temp_likelihood = 1.0
            likelihood_arrTasks = np.array([])
            likelihood_arrSubTasks = np.array([])
            # Calculate the likelihood of the user observations in the current model
            for ind in range(len(curr_human_obs)):
                # Mapping observable IDs to action IDs.
                action = -1
                subtaskEnded = False
                obs = curr_human_obs[ind]
                if obs == 33: action = 2
                elif obs == 17: action = 4
                elif obs == 5:
                    action = 0
                    grasp_status = 1
                elif obs == 9:
                    action = 0
                    grasp_status = 0
                
                # Extracting in which states that action might be selected according to the policy
                index = np.array([])
                if action == 2 or action == 1 or action == 3: # mapping look around and walkaway into idle action
                    #action = np.array([1,2,3]) 
                    index1 = np.where(policy == 1) # looking around
                    index2 = np.where(policy == 2) # idle
                    index3 = np.where(policy == 3) # walking away
                    index = np.append(np.array(index1[0]), np.array(index2[0]))
                    index = np.append(index, np.array(index3[0]))
                else:
                    index = np.append(index, np.where(policy == action))
                curr_act_prob = np.zeros(12)
                    
                for i in index:
                    curr_act_prob[int(i)] = 1
                if (not 0 in policy) and (not 1 in curr_act_prob):
                    curr_act_prob[5] = 1 # evaluating state might lead to grasping
                    curr_act_prob[3] = 1 # failed to grasp state might lead to grasping
                #print(curr_act_prob)
                
                # Checking if a subtask has ended. will be used later as the model should be reset.
                if not simu_flag: # in data from real humans, we dont have markers that indicates when a subtask has started                
                    if action == 0: # at least from the human side, we know that if human has graspped a subtask ends. But robot side is unknown
                        subtaskEnded = True
                else:
                    if ((ind != len(curr_subtaskID_arr) - 1) and (curr_subtaskID_arr[ind] != curr_subtaskID_arr[ind + 1])): # subtask has ended, saving the average subtask likelihoods as the task likelihood and restarting
                        subtaskEnded = True
                
                # LIKELIHOOD CALCULATION
                curr_likelihood = 0.0
                for state in range(len(curr_belief)):
                    curr_likelihood += curr_belief[state] * curr_act_prob[state]
                temp_likelihood *= curr_likelihood
                if (curr_likelihood == 0.0): # That means there is an uncovered (unknown) observation
                    print("index:", ind, "action:", action, "act_prob:", curr_act_prob)
                
                # BELIEF UPDATE: current_belief * T(a,s)
                temp_belief = np.zeros(12)
                trans_action = T[action]
                for state in range(len(curr_belief)):
                    temp_belief[state] = np.matmul(curr_belief, np.transpose(trans_action[:,state]))
                curr_belief = temp_belief
                
                # if a subtask is a failure or a success, a new likelihood should be calculated. Then it terminates a subtask
                if subtaskEnded:
                    if grasp_status == 1:
                        curr_act_prob = [0,1,0,0,0,0,0,0,0,0,0,0] # global success should be the new likelihood
                    elif grasp_status == 0:
                        curr_act_prob = [0,0,1,0,0,0,0,0,0,0,0,0] # global fail and failed to grasp should be the new likelihood
                    curr_likelihood = 0.0
                    for state in range(len(curr_belief)):
                        curr_likelihood += curr_belief[state] * curr_act_prob[state]
                    temp_likelihood *= curr_likelihood
                    # saving the likelihood of a subtask, then restarting the belief and calculations
                    # reset variables
                    likelihood_arrSubTasks = np.append(likelihood_arrSubTasks, temp_likelihood)
                    curr_belief = init_belief
                    temp_likelihood = 1.0
                    grasp_status = -1 

                #else: # if subtask hasn't ended but the person has graspped and failed, the next state will definitely be Failed To Grasp
        
                # If a task has ended, reset the current belief and save the average task likelihood
                if (((ind != len(curr_taskID_arr) - 1) and curr_taskID_arr[ind] != curr_taskID_arr[ind + 1]) or (ind == len(curr_taskID_arr) - 1)): # task has ended, saving the average subtask likelihoods as the task likelihood and restarting
                    curr_belief = init_belief
                    #print("Task/subtask update!, Task:", ind, "Likelihood:", likelihood)
                    average_task_likelihood = np.average(likelihood_arrSubTasks)
                    # Do not save the task IDs "1" or "5" as they are tasks with participants working alone. No robot. So wont be considered
                    if (not simu_flag) and (curr_taskID_arr[ind] != 1 and curr_taskID_arr[ind] != 5): 
                        likelihood_arrTasks = np.append(likelihood_arrTasks, average_task_likelihood)
                    elif simu_flag:
                        likelihood_arrTasks = np.append(likelihood_arrTasks, average_task_likelihood)
                    
                    likelihood_arrSubTasks = np.array([])
                    temp_likelihood = 1.0
                            
            # Append the user's likelihood array for each tasks for the current model. The array below will be 8 x 6 (# of models X # of tasks)
            #likelihood_arrUser = np.concatenate((likelihood_arrUser, likelihood_arrTasks), axis=0)
            if (simu_flag):
                likelihood_arrParticipants[model_id][user_id] = np.average(likelihood_arrTasks)
            else:
                likelihood_arrUser[model_id] = likelihood_arrTasks  
        #print(likelihood_arrUser)
        # Append the participant array holding all the participant's likelihood w.r.t to each human model for each task
        if not simu_flag:
            likelihood_arrParticipants.append(likelihood_arrUser)
    
    print(likelihood_arrParticipants)
    print(policy)
    return likelihood_arrParticipants
    
if __name__=='__main__':
    
    #user_obs, user_taskID_arr, simu_obs, simu_taskID_arr, simu_subtaskID_arr = readObs()
    
    my_shelf = shelve.open("user_simu_obs.out")
    user_obs = my_shelf['user_obs']
    simu_obs = my_shelf['simu_obs']
    user_taskID_arr = my_shelf['user_taskID_arr']
    simu_taskID_arr = my_shelf['simu_taskID_arr']
    simu_subtaskID_arr = my_shelf['simu_subtaskID_arr']
    
    #participants_likelihood = MDPmodelLikelihood(False)
    simulation_likelihood = MDPmodelLikelihood(True)
    
    # Converting Participant likelihood array for each task to maximum scores
    # TODO:    
    
    #print(score)
    

    