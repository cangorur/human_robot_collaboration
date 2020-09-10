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
import csv
import sys

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

def readObs_user():
    # Reading real human data collected from the participants
    global user_obs, user_taskID_arr
    
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
#                    if observation_number == 17:
#                        user_obs[user_id] = np.insert(user_obs[user_id], obs, 17)
#                        user_obs[user_id] = np.insert(user_obs[user_id], obs, 33)
#                        user_taskID_arr[user_id] = np.insert(user_taskID_arr[user_id], obs, user_taskID_arr[user_id][obs])
#                        user_taskID_arr[user_id] = np.insert(user_taskID_arr[user_id], obs, user_taskID_arr[user_id][obs])
#                        user_obs[user_id][obs+2] = observation_number
#                        obs += 3                     
                    user_obs[user_id][obs] = observation_number
                
                
                    
    my_shelf = shelve.open("user_simu_obs.out") # 'n' for new    
    my_shelf["user_obs"] = user_obs
    my_shelf["user_taskID_arr"] = user_taskID_arr
    my_shelf.close()         
    
    return user_obs, user_taskID_arr


def readObs_simu():
    
    global simu_obs, simu_taskID_arr, simu_subtaskID_arr
    
    dir_path = os.path.dirname(os.path.realpath(__file__))

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
        temp_obs = np.array([])
        temp_task = np.array([])
        temp_subtask = np.array([])
        grasp_complete = False
        grasp_ctr = 0
        grasp_state = ""
        idle_ctr = 0
        warn_ctr = 0
        prev_obs = 33
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
           
            
             # Once a grasp is detected, there cannot be a new one coming in a subtask. We remove all of them but keep the latest one by setting this flag if they are in the same subtask.
            if obs!=0 and simu_subtaskID_arr[simu_id][obs] != simu_subtaskID_arr[simu_id][obs-1]:
                grasp_complete = False
                grasp_ctr = 0
                idle_ctr = 0
                warn_ctr = 0
            elif (obs != len(simu_obs[simu_id]) -1) and (simu_subtaskID_arr[simu_id][obs] != simu_subtaskID_arr[simu_id][obs+1]):
                if grasp_state == "success":
                    observation_number = 5
                if grasp_state == "failure":
                    observation_number = 9
                    temp_obs = np.append(temp_obs, 33)                        
                    temp_task = np.append(temp_task, simu_taskID_arr[simu_id][obs])
                    temp_subtask = np.append(temp_subtask, simu_subtaskID_arr[simu_id][obs])
                    temp_obs = np.append(temp_obs, 9)                        
                    temp_task = np.append(temp_task, simu_taskID_arr[simu_id][obs])
                    temp_subtask = np.append(temp_subtask, simu_subtaskID_arr[simu_id][obs])
#                temp_obs = np.append(temp_obs, 33)                        
#                temp_task = np.append(temp_task, simu_taskID_arr[simu_id][obs])
#                temp_subtask = np.append(temp_subtask, simu_subtaskID_arr[simu_id][obs])
#                temp_obs = np.append(temp_obs, observation_number)                        
#                temp_task = np.append(temp_task, simu_taskID_arr[simu_id][obs])
#                temp_subtask = np.append(temp_subtask, simu_subtaskID_arr[simu_id][obs])
            
            if grasp_complete: # grasp_complete states a grasp has already been issued in this subtask. 
                if observation_number == 5:
                    observation_number = 33
                    grasp_state = "success"
                if observation_number == 9:
                    observation_number = 33
                    grasp_state = "failure"

                grasp_ctr += 1

            elif grasp_complete == False and (observation_number == 5 or observation_number == 9):
                grasp_complete = True
                grasp_ctr += 1
            
                #observation_number = 33
#                temp_obs = np.append(temp_obs, 33)                        
#                temp_task = np.append(temp_task, simu_taskID_arr[simu_id][obs])
#                temp_subtask = np.append(temp_subtask, simu_subtaskID_arr[simu_id][obs])
            
            if observation_number == 17 and prev_obs == 17:
                #observation_number = 33
                warn_ctr += 1
            elif observation_number == 33 and prev_obs == 33:
                idle_ctr += 1
#                temp_obs = np.append(temp_obs, 33)                        
#                temp_task = np.append(temp_task, simu_taskID_arr[simu_id][obs])
#                temp_subtask = np.append(temp_subtask, simu_subtaskID_arr[simu_id][obs])
                
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
            
            prev_obs = observation_number
            if (grasp_complete and grasp_ctr > 1) or idle_ctr > 2 or warn_ctr > 2:
                continue 
            temp_obs = np.append(temp_obs, observation_number)
            temp_task = np.append(temp_task, simu_taskID_arr[simu_id][obs])
            temp_subtask = np.append(temp_subtask, simu_subtaskID_arr[simu_id][obs])
            
        simu_obs[simu_id] = temp_obs
        simu_taskID_arr[simu_id] = temp_task
        simu_subtaskID_arr[simu_id] = temp_subtask

    my_shelf = shelve.open("user_simu_obs.out") # 'n' for new    
    my_shelf["simu_obs"] = simu_obs
    my_shelf["simu_taskID_arr"] = simu_taskID_arr
    my_shelf["simu_subtaskID_arr"] = simu_subtaskID_arr
    my_shelf.close()
    #savemat("updated_user_simu_obs.mat",{'user_obs':np.array(user_obs), 'simu_obs':np.array(simu_obs),
     #                                  'user_taskID_arr':np.array(user_taskID_arr),'simu_taskID_arr':np.array(simu_taskID_arr)})
    
    return simu_obs, simu_taskID_arr, simu_subtaskID_arr
    
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
    
    model_ids = []
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
    dir_path = os.path.dirname(os.path.realpath(__file__))

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
        
        with open(dir_path + '/simulation_likelihoods_' + str(user_id) + '.csv', 'a') as file:
            wr = csv.writer(file, dialect='excel')
            
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
                grasp_status = -1
                
                instance_ctr = 0
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
                    instance_ctr += 1
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
                        if action == 0 or (instance_ctr > 3 and ind+1 != len(curr_human_obs) and curr_human_obs[ind+1] == 33): # at least from the human side, we know that if human has graspped a subtask ends. But robot side is unknown
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
                        instance_ctr = 0
    
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
                
                wr.writerow(likelihood_arrTasks)
            #print(likelihood_arrUser)
            # Append the participant array holding all the participant's likelihood w.r.t to each human model for each task
            if not simu_flag:
                likelihood_arrParticipants.append(likelihood_arrUser)
            
            file.close()
    
    print(likelihood_arrParticipants)
    print(policy)
    return likelihood_arrParticipants
    
def likelihood_analysis(participants_likelihood):
#    
#    li_shelf = shelve.open("likelihoods.out")
#    simulation_likelihood = my_shelf['simulation_likelihood']
#    participants_likelihood = my_shelf['participants_likelihood']
    
    dataset=loadmat("mdp_likelihood_fullData_v2.mat")
    sim_like=dataset['simulation_likelihood']
    part_like=dataset['participants_likelihood']
    part_like = participants_likelihood
    avg_part_like = np.zeros((14, 8))
    for user in range(len(part_like)):
        for model in range(len(part_like[user])):
            avg_part_like[user][model] = np.average(part_like[user][model])
    
    return avg_part_like

if __name__=='__main__':
    
    #user_obs, user_taskID_arr = readObs_user()
    #simu_obs, simu_taskID_arr, simu_subtaskID_arr = readObs_simu()
    
    my_shelf = shelve.open("user_simu_obs.out")
    user_obs = my_shelf['user_obs']
    simu_obs = my_shelf['simu_obs']
    user_taskID_arr = my_shelf['user_taskID_arr']
    simu_taskID_arr = my_shelf['simu_taskID_arr']
    simu_subtaskID_arr = my_shelf['simu_subtaskID_arr']
    my_shelf.close()
#    
    #participants_likelihood = MDPmodelLikelihood(False)
    #simulation_likelihood = MDPmodelLikelihood(True)
    
#    li_shelf = shelve.open("likelihoods.out",'n') # 'n' for new    
#    li_shelf["simulation_likelihood"] = simulation_likelihood
#    li_shelf["participants_likelihood"] = participants_likelihood
#    li_shelf.close()

    #avg_participants_likelihood = likelihood_analysis(participants_likelihood)    
    avg_simu_likelihoods = np.array([[0.2268696682, 0.226, 0.1777611125, 0.1264305124,0.052634886, 0.0553250766,0.0493724096,0.047642683],
    [0.205,	0.223,	0.1545213063,0.1125685778,0.0512893127,0.0549212468,0.044792838,0.0458228771],
    [0.1870026504,0.1794263463,0.215,0.0906976491,0.04735648,0.0390272992,0.0596455864,0.0443862323],
    [0.1761746375,0.1709018503,0.1079432919,0.212,0.0422650048,0.036249426,0.0480043587,0.0388828557],
    [0.1282663103,0.1291115353,0.0882418617,0.07800314,0.185,0.0696268856,	0.0435897361,	0.058018806],
    [0.1323877723,	0.1331554949,	0.0896511713,	0.0787243066,	0.0658237581,	0.175,	0.0433316746,	0.0575937871],
    [0.2126987958,	0.189992599,	0.1864392941,	0.1440196239,	0.0828012341,	0.0516418829,	0.228,	0.0818530789],
    [0.106786256,	0.1012134949,	0.0829620657,	0.0675724011,	0.0408817896,	0.0328882159,	0.0492111886,	0.117]    
    ])

    # Converting Participant likelihood array for each task to maximum scores
    # TODO:    
    
    #print(score)
    

    