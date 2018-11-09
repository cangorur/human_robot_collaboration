#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  6 12:05:55 2018
@author: gunerdilsader
"""

import numpy as np
from scipy.io import savemat
from hrc_ros.msg import TaskState
import json
import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import String


# @TODO:json file directory should be changed!
json_file="../../../../../configs/scenario_config.json"

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

std_model = np.zeros((tCount, pCount))
series_std=np.zeros((N, tCount, pCount))
mu_model = np.zeros((tCount, pCount))
series_mu=np.zeros((N, tCount, pCount))

# Just to check whether we had the same result.
std_model1 = np.zeros((tCount, pCount))
mu_model1 = np.zeros((tCount, pCount))


observation_model=[[0 for _ in range(len(humtypes))] for _ in range(len(policies)) ]
    # this 2-D matrix is a basis for observation model, it should be filled by
    # observation rows with respect to each (human_type , policy) couple
observation_number=0

observation_vector=np.array([])
observation_signal=[]
reward=np.array([])
last_reward_signal=0

n=-1

def observation_update(data):
    #observation comes from another ros agent, make it useful
    #global new_task_arrived
    #global current_task_id
    global observation_vector
    global observation_signal
    global last_reward_signal

    global observation_model
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

    if (data.who_reports == 'MANAGER' and data.task_status == 'START'):
        # If new task starts; propagate observation and reward
        # Then initilize them.

        policies_array=np.array(policies)
        humtypes_array=np.array(humans)

        [[t]]=np.where(humtypes_array==data.human_model)
        [[p]]=np.where(policies_array==data.robot_model)

        if n==N-1:
            n=0
            mu_model[t,p]=np.average(series_mu[:,t,p])
            std_model[t,p]=np.std(series_std[:,t,p])
        else:
            n=n+1

    if (data.who_reports == 'OBSERVATION'):
        # collecting observation
        instant_observation_str=data.human_observables
        instant_observation=np.zeros(len(data.human_observables))
        for i in range(0,len(data.human_observables)):
            instant_observation[i]=int(bool(instant_observation_str[i]))

        observation_signal.append(instant_observation)

#def reward_update(self,instant_reward):
    #reward comes from another ros agent, make it useful
    if (data.who_reports == 'ROBOT' and (data.real_state == 'GlobalSuccess' or data.real_state == 'GlobalFail')):

        last_reward_signal=data.total_disc_reward
        rospy.loginfo("New task started, old data is appending")

        observation_vector=np.array(observation_signal)
        reward=np.array(last_reward_signal)

        # We need observation_vector and reward resulted from a tasks
        # Subscribe TaskStatus topic and take them via observation_update()
            # observation: all observations during one episode
            # reward: total reward comes from one episode
        # observation_vector=observation_array
        # reward=last_reward_signal
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

        row=np.array(observation_row)/N
        observation_model[p][t]=row

        series_std[n,t,p]=reward
        series_mu[n,t,p]=reward

        mu_model1[t,p]=np.average(series_mu[:,t,p])
        std_model1[t,p]=np.std(series_std[:,t,p])

        rospy.loginfo(data.task_id-1,observation_vector,reward)
        # initilize obervation an reward
        reward=np.array([])
        observation_signal=[]
        rospy.loginfo(n,t,p)

        # when the sets are ready :
        if p==pCount-1 and t==tCount-1 and n==N-1:

            print("===================== Saving into .mat file ========================")
            observation_model=np.array(observation_model)
            savemat(Training_Set_Name,{'policies':policies, 'humtypes':humans,'mu_model':mu_model ,'std_model':std_model,'mu_model1':mu_model1 ,'std_model1':std_model1 ,'observation_model':observation_model,'num_of_observables':num_of_obs})
            print("===================== Saved into .mat file ========================")


def Trainer(request):

    rospy.Subscriber("/task_manager/task_status", TaskState, observation_update)
    return[True,"Ready"]

if __name__ == "__main__":
    rospy.init_node("BPR_trainer", anonymous=True)
    rospy.loginfo("Training Phase is ready!")
    train_algorithm_service= rospy.Service('~/train_algorithm',Trigger, Trainer)
    rospy.spin()
