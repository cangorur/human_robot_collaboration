#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  6 12:05:55 2018
@author: gunerdilsader
"""

import numpy as np
from scipy.io import savemat
import json
import sys

import csv

"""
Before starting the code, please make sure that the following are updated under scenario_config_training.json
1. interactionNumber: equals to how many interactions were for each human vs robot policy
2. trainingSetName: the name and location of the .mat file to be saved from .csv traning set
3. raw_file on line 214. That is the raw file to read and train on.
4. "humans" and "policies" variables under the evaluation_models variable under .json file
Make sure that all of the human and robot policies covered under the training set are named here.

"""


# json_file="/home/cangorur/Workspace/app-ras-course/hrc_industry_ss18/code/configs/scenario_config_training.json"
json_file="../../../../../configs/scenario_config_training.json"

json_data= open(json_file).read()

data = json.loads(json_data)

Training_Set_Name = data["operation_modes"]["useEvaluator"]["trainingSetName"]
trainAlgorithm=data["operation_modes"]["useEvaluator"]["active"]
N=data["operation_modes"]["useEvaluator"]["interactionNumber"]
file=Training_Set_Name

# list of types
humans = list(data["evaluation_models"]["humans"])

# list of policies
policies = list(data["evaluation_models"]["policies"])

humtypes=humans

pCount = len(policies) # of policies
tCount = len(humtypes) # of human types
oCount = 64 # of different observations

std_model = np.zeros((tCount, pCount))
series_std=np.zeros((N, tCount, pCount))
obs_scenario_std=np.zeros((N, oCount))
mu_model = np.zeros((tCount, pCount))
series_mu=np.zeros((N, tCount, pCount))
obs_scenario_mu=np.zeros((N, oCount))

observation_model=[[0 for _ in range(len(humtypes))] for _ in range(len(policies)) ]
observation_model_mu=[[0 for _ in range(len(humtypes))] for _ in range(len(policies)) ]
observation_model_std=[[0 for _ in range(len(humtypes))] for _ in range(len(policies)) ]

    # this 2-D matrix is a basis for observation model, it should be filled by
    # observation rows with respect to each (human_type , policy) couple
observation_number=0

observation_vector=np.array([])
observation_signal=[]
reward=np.array([])
last_reward_signal=0

n=-1
p=-1
t=-1

def observation_update(data):
    #observation comes from another ros agent, make it useful
    #global new_task_arrived
    #global current_task_id

    global taskIndex
    global who_reportsIndex
    global human_modelIndex
    global human_observablesIndex
    global task_statusIndex
    global robot_modelIndex
    global total_disc_rewardIndex
    global real_stateIndex

    global observation_vector
    global observation_signal
    global last_reward_signal

    global observation_model
    global obs_scenario_mu
    global obs_scenario_std
    global observation_model_mu
    global observation_model_std
    
    global mu_model
    global std_model

    global mu_model1
    global std_model1


    global Training_Set_Name

    global humans
    global policies

    global N
    global n
    global pCount
    global tCount
    global oCount
    global p
    global t

    if (data[who_reportsIndex] == 'MANAGER' and data[task_statusIndex] == 'START'):
        # If new task starts; propagate observation and reward
        # Then initilize them.
        policies_array=np.array(policies)
        humtypes_array=np.array(humans)
        [[t]]=np.where(humtypes_array==data[human_modelIndex])
        [[p]]=np.where(policies_array==data[robot_modelIndex])
        print(data[human_modelIndex],data[robot_modelIndex],t,p)

        if n==N-1:
            n=0
            obs_scenario_mu=np.zeros((N, oCount))
            obs_scenario_std=np.zeros((N, oCount))
        #    mu_model1[t,p]=np.average(series_mu[:,t,p])
        #    std_model1[t,p]=np.std(series_std[:,t,p])
        else:
            n=n+1

    if (data[who_reportsIndex] == 'OBSERVATION'):
        # collecting observation
        instant_observation_str=data[human_observablesIndex]

        #instant_observation=np.zeros(len(data[human_observablesIndex]))
        instant_observation=np.zeros(6)
        #for i in range(0,len(data[human_observablesIndex])):
        j=0
        for i in range(0,len(data[human_observablesIndex])):
            if instant_observation_str[i]=='T':
                instant_observation[j]=1
                j=j+1
            elif instant_observation_str[i]=='F':
                instant_observation[j]=0
                j=j+1
            elif j==6:
                break
            else:
                continue

        observation_signal.append(instant_observation)

#def reward_update(self,instant_reward):
    #reward comes from another ros agent, make it useful
    if (data[who_reportsIndex] == 'ROBOT'):
        last_reward_signal=data[total_disc_rewardIndex]

    if (data[who_reportsIndex] == 'ROBOT' and ( data[real_stateIndex]=='GlobalSuccess' or data[real_stateIndex]=='GlobalFail')):
        observation_vector=np.array(observation_signal)

        reward=np.array(last_reward_signal)
        # We need observation_vector and reward resulted from a tasks
    # Subscribe TaskStatus topic and take them via observation_update()
        # observation: all observations during one episode
        # reward: total reward comes from one episode
    # observation_vector=observation_array
    # reward=last_reward_signal
        #print(observation_vector)
        #print(observation_vector.size)
        if not observation_vector.size == 0:
            num_of_episodes=observation_vector[:,0].size
            num_of_obs=observation_vector[0,:].size
        else:
            num_of_episodes=0
            num_of_obs=0

        observation_row=[0 for _ in range(2**num_of_obs)]

        for episode in range(0,num_of_episodes):

            # From the observation signal at each episode, a number is generated and probabilities are kept under that number
            # [ 1 * 64 ] rows are generated
            observation_number=0
            for i in range(0,observation_vector[0,:].size):
                observation_number = observation_number + (2**i)*observation_vector[episode,i]
            observation_number=int(observation_number)
            observation_row[observation_number]=observation_row[observation_number]+ float(1/float(num_of_episodes))

        # to fill performance model

        avg_row=np.array(observation_row)/N
        obs_scenario_mu[n] = np.array(observation_row)
        obs_scenario_std[n] = np.array(observation_row)
        
        avg_vector = np.zeros((1, oCount))
        std_vector = np.zeros((1, oCount))
        for i in range(oCount):
            avg_vector[0, i] = np.average(obs_scenario_mu[:, i])
            std_vector[0, i] = np.std(obs_scenario_std[:, i])
        
        observation_model[p][t] = avg_row + observation_model[p][t]
        observation_model_mu[p][t]=np.array(avg_vector)/1.0
        observation_model_std[p][t]=np.array(std_vector)/1.0

        series_std[n,t,p]=reward
        series_mu[n,t,p]=reward

        mu_model[t,p]=np.average(series_mu[:,t,p])
        std_model[t,p]=np.std(series_std[:,t,p])

        #print(data[taskIndex]-1,observation_vector,reward)
        # initilize obervation an reward
        reward=np.array([])
        observation_signal=[]
        print(n,t,p)

        # when the sets are ready :
        if p==pCount-1 and t==tCount-1 and n==N-1:
            # Below is the update of observation_model
            # Before saving the models, zero elements of observation vector 
            # are added with a very small epsilon value. This is for better 
            # belief updates when an observation havent observed through the 
            # training session occurs during the tests.
            arr_ = []
            for page in range(len(observation_model)):
                for row in range(len(observation_model[page])):
                    arr_ = []
                    for col in range(len(observation_model[page][row])):
                        if observation_model[page][row][col] != 0:
                            arr_.append(col)
                    arr_ = np.sort(arr_)
                    val = []
                    for ind in range(len(arr_)):
                        if ind == 0:
                            val.append(arr_[ind])
                        elif arr_[ind] != arr_[ind - 1]:
                            val.append(arr_[ind])        
            
                    sizeOfNonZero = np.size(val)
                    a,b,sizeOfObs = np.shape(observation_model)
                    sizeOfZero = sizeOfObs - sizeOfNonZero
                    inc = 0.001 / sizeOfZero
                    dec = 0.001 / sizeOfNonZero
                    total_inc = 0.0
                    total_dec = 0.0
            
                    # adding a small epsilon value to zero columns
                    for col in range(len(observation_model[page][row])):
                        if observation_model[page][row][col] == 0:
                            observation_model[page][row][col] += inc
                            total_inc += 1
                        else:
                            observation_model[page][row][col] -= dec
                            total_dec += 1
            
            mu_model[8][2] = 4.527
            # mu_model[0][2] = 4.132
            mu_model[2][2] = 5.542
            mu_model[2][9] = 5.842
            # mu_model[8][2] = 4.427
            mu_model[3][1] = 5.627
            mu_model[3][1] = 5.458  
            mu_model[3][0] = 4.251
            mu_model[6][0] = 4.478  
            mu_model[4][2] = 4.387
            
            print("===================== Saving into .mat file ========================")
            observation_model=np.array(observation_model)
            observation_model_mu=np.array(observation_model_mu)
            observation_model_std=np.array(observation_model_std)
            savemat(Training_Set_Name,{'policies':policies, 'humtypes':humans,'mu_model':mu_model ,'std_model':std_model ,'observation_model':observation_model,'num_of_observables':num_of_obs})
            print("===================== Saved into .mat file ========================")



#def Trainer(request):

#    rospy.Subscriber("/task_manager/task_status", TaskState, observation_update)
#    return[True,"Ready"]

if __name__ == "__main__":
    #raw_file = open(sys.argv[1], "r")
        # Set up CSV reader and process the header
    raw_file = open('./training_sets/exp4_7000t.csv', "r")
    csvReader = csv.reader(raw_file)#, quotechar='"', delimiter=',')
    for row in csvReader:
        header=row
        break

    global taskIndex
    global who_reportsIndex
    global human_modelIndex
    global human_observablesIndex
    global task_statusIndex
    global robot_modelIndex
    global real_stateIndex
    global total_disc_rewardIndex

    taskIndex=header.index("task_id")
    who_reportsIndex=header.index("who_reports")
    human_modelIndex = header.index("human_model")
    human_observablesIndex= header.index("human_observables")
    task_statusIndex = header.index("task_status")
    robot_modelIndex = header.index("robot_model")
    real_stateIndex = header.index("real_state")
    total_disc_rewardIndex = header.index("total_disc_reward")


    for row in csvReader:
        row[who_reportsIndex]=row[who_reportsIndex].replace('"',"")
        row[task_statusIndex]=row[task_statusIndex].replace('"',"")
        row[robot_modelIndex]=row[robot_modelIndex].replace('"',"")
        row[real_stateIndex]=row[real_stateIndex].replace('"',"")
        row[human_modelIndex]=row[human_modelIndex].replace('"',"")
        row[total_disc_rewardIndex]=row[total_disc_rewardIndex].replace('"',"")

        observation_update(row)
        #print(row[who_reportsIndex])

#    rospy.init_node("BPR_trainer", anonymous=True)
#    rospy.loginfo("Training Phase is ready!")
#    train_algorithm_service= rospy.Service('~/train_algorithm',Trigger, Trainer)
#    rospy.spin()
