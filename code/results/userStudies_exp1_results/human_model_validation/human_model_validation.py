#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 24 23:45:29 2020

@author: cangorur
"""

import sklearn.metrics as mtr
import pandas as pd
import os
import numpy as np

user_obs = []
simu_obs = []
user_taskID_arr = []
simu_taskID_arr = []

def readObs():
    # Reading real human data collected from the participants
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
            if (user_obs[user_id][obs] == 41 or user_obs[user_id][obs] == 43 or user_obs[user_id][obs] == 40):
                user_obs[user_id][obs] = 9
            elif (user_obs[user_id][obs] == 25):
                user_obs[user_id][obs] = 17
    
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
    

if __name__=='__main__':
    
    final_kl_array_max = []
    final_kl_array = []
    user_obs, user_taskID_arr, simu_obs, simu_taskID_arr = readObs()
    user_obs = removeRepeatingObs(user_obs)
    simu_obs = removeRepeatingObs(simu_obs)    
    final_kl_array, final_kl_array_max = KLConvergenceAnalysis(user_obs, user_taskID_arr, simu_obs, simu_taskID_arr)
    
    final_kl_array_max = np.array(final_kl_array_max)
    print(final_kl_array_max)
    #print(score)
    