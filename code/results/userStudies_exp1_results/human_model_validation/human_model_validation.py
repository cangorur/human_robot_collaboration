#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 24 23:45:29 2020

@author: cangorur
"""

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
list_of_models = []
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
    global user_obs, simu_obs, user_taskID_arr, simu_taskID_arr
    
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
        for obs in range(len(user_obs[user_id])):
            observation_number = user_obs[user_id][obs]
            if (observation_number == 41 or observation_number == 43 or observation_number == 40 or observation_number == 11):
                observation_number = 9
            elif (observation_number == 25 or observation_number == 49 or observation_number == 21 or observation_number == 57 or observation_number == 16 or observation_number == 19 or observation_number == 18):
                observation_number = 17
            elif (observation_number == 7 or observation_number == 37):
                observation_number = 5
            elif (observation_number == 32 or observation_number == 1):
                observation_number = 33
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
        simu_id += 1
        
     # The mappings are necessary for real observations as a stood idle observable always high after a failure or success in grasp is issued. In simulation it is not the case but they reflect the same scenario    
    for simu_id in range(len(simu_obs)):
        for obs in range(len(simu_obs[simu_id])):
            observation_number = simu_obs[simu_id][obs]
            if (observation_number == 41 or observation_number == 43 or observation_number == 40 or observation_number == 11):
                observation_number = 9
            elif (observation_number == 25 or observation_number == 49 or observation_number == 21 or observation_number == 57 or observation_number == 16 or observation_number == 19 or observation_number == 18):
                observation_number = 17
            elif (observation_number == 7 or observation_number == 37):
                observation_number = 5
            elif (observation_number == 32 or observation_number == 1): # 2, 34, 35, 3 are all look around 
                observation_number = 33
            simu_obs[simu_id][obs] = observation_number
    
    return user_obs, user_taskID_arr, simu_obs, simu_taskID_arr
    
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
    
def MDPpolicyGeneration():
    
    global list_of_models
    
    readHumanModel()
    model_tree = list_of_models[0]
    P, R, discount, init_belief = readProbTables(model_tree)
    
    mdptoolbox.util.check(P, R)
    vi = mdptoolbox.mdp.ValueIteration(P, R, discount)
    vi.run()
    print(vi.policy)
    print(vi.V)
    
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
    global list_of_models
    
    path = '../../../models/human_models/Evaluate/'
    expertise = ["beginner", "expert"]
    stamina = ["tired", "nontired"]
    col = ["collaborative", "noncollaborative"]    
    filename = expertise[0] + "_" + stamina[1] + "_" + col[0] + ".POMDPx"    
    pomdpx_file = path + filename
    #print('loading... %s' %pomdpx_file)
    tree = ET.parse(pomdpx_file)
    list_of_models.append(tree)
    
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

if __name__=='__main__':
    
    final_kl_array_max = []
    final_kl_array = []
    #user_obs, user_taskID_arr, simu_obs, simu_taskID_arr = readObs()
    #user_obs = removeRepeatingObs(user_obs)
    #simu_obs = removeRepeatingObs(simu_obs)    
    #final_kl_array, final_kl_array_max = KLConvergenceAnalysis(user_obs, user_taskID_arr, simu_obs, simu_taskID_arr)
    
    #final_kl_array_max = np.array(final_kl_array_max)
    #print(final_kl_array_max)
    MDPpolicyGeneration()
    #print(score)
    
    
    #dataset1=loadmat("kl_convergence_slidingWindow_fullData_v2.mat")
    #arr = dataset1['final_kl_array_max']
    #dataset1['final_kl_array_max'] = np.array(dataset1)
    #savemat("kl_convergence_slidingWindow_fullData_v2.mat",dataset1)

    