#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 24 23:45:29 2020

@author: cangorur
"""

import sklearn.metrics as mtr
import pandas as pd
import os

user_obs = []
taskID_arr = []

def readObs():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    df = pd.read_excel(dir_path + '/../userStudies_exp2_results/tests/analysis_objective.xlsx', sheet_name='human_observables', sep='\s*,\s*')
    part_id = 4
    total_subjects = 14
    for i in range(total_subjects-1):
        part_temp_obs = df['Obs_' + str(part_id)].values.tolist()
        part_temp_obs = [x for x in part_temp_obs if str(x) != 'nan']
        user_obs.append(part_temp_obs)
        part_tasks = df['task_id_' + str(part_id)].values.tolist()
        part_tasks = [x for x in part_tasks if str(x) != 'nan']
        taskID_arr.append(part_tasks)
        part_id += 1
    return user_obs, taskID_arr

if __name__=='__main__':
    
    user_obs,taskID_arr = readObs()
    score = mtr.mutual_info_score(user_obs[1][:256],user_obs[2])
    print(score)
    