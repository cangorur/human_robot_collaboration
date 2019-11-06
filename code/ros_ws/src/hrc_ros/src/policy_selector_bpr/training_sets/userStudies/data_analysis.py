#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 26 17:20:57 2019

@author: cangorur
"""

from scipy.io import loadmat, savemat
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import json
import csv
import os


"""
plt.figure()
    
for i in range(0,10):
    s=obs2[0,i]
    plt.plot(range(0,64),s)
    


plt.legend(("distracted_collaborative",
        "expert_nontired_collaborative",
        "expert_nontired_noncollaborative",
        "beginner_tired_noncollaborative",
        "beginner_tired_collaborative",
        "beginner_nontired_noncollaborative",
        "expert_tired_collaborative",
        "expert_tired_noncollaborative",
             "beginner_nontired_collaborative",
         "default.POMDPx"),loc=1)

plt.grid()

plt.annotate('successful', xy=(5, 0.34), xytext=(0, 0.4),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )


plt.annotate('lookaround&idle', xy=(35, 0.24), xytext=(20, 0.34),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )

plt.annotate('walk away', xy=(0, 0.24), xytext=(0, 0.3),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )
plt.annotate('idle', xy=(33, 0.20), xytext=(20, 0.24),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )
plt.annotate('warn', xy=(17, 0.10), xytext=(10, 0.14),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )
plt.annotate('fail', xy=(9, 0.20), xytext=(10, 0.24),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )
plt.title("proactive_beginner.pomdpx")


plt.figure()
    
for i in range(0,10):
    s=obs2[2,i]
    plt.plot(range(0,64),s)
    

plt.legend(("distracted_collaborative",
        "expert_nontired_collaborative",
        "expert_nontired_noncollaborative",
        "beginner_tired_noncollaborative",
        "beginner_tired_collaborative",
        "beginner_nontired_noncollaborative",
        "expert_tired_collaborative",
        "expert_tired_noncollaborative",
             "beginner_nontired_collaborative",
         "default.POMDPx"),loc=1)

plt.grid()

plt.annotate('successful', xy=(5, 0.34), xytext=(0, 0.4),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )
plt.annotate('lookaround&idle', xy=(35, 0.24), xytext=(20, 0.34),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )

plt.annotate('walk away', xy=(0, 0.24), xytext=(0, 0.3),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )
plt.annotate('idle', xy=(33, 0.20), xytext=(20, 0.24),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )
plt.annotate('warn', xy=(17, 0.10), xytext=(10, 0.14),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )
plt.annotate('fail', xy=(9, 0.20), xytext=(10, 0.24),
            arrowprops=dict(facecolor='black', shrink=0.05),
            )
plt.title("proactive_distracted.pomdpx")
"""

def bayesian_estimation(obs_f, num_obs, humtypes):
    
    # In case it is used. obs_f is averaged over all the policies (also excluding reminder models)
    
    '''
    with open(dir_path + '/../../../../../../../results/userStudies_exp1_results/human_observables.csv', 'rb') as file:
        reader = csv.reader(file, delimiter=',')
        real_obs = list(reader)
        print real_obs[2][7]
    '''
    # reading the obs as arrays for each participant data
    df = pd.read_excel(dir_path + '/../../../../../../../results/userStudies_exp1_results/analysis_objective.xlsx', sheet_name='human_observables')
    part_id = 4
    user_obs = []
    taskID_arr = []
    for i in range(14):
        part_temp_obs = df['Obs_' + str(part_id)].values.tolist()
        part_temp_obs = [x for x in part_temp_obs if str(x) != 'nan']
        user_obs.append(part_temp_obs)
        part_tasks = df['task_id_' + str(part_id)].values.tolist()
        part_tasks = [x for x in part_tasks if str(x) != 'nan']
        taskID_arr.append(part_tasks)
        part_id += 1
    
    # initializing the variables
    observation_model=obs_f
    beliefSet = np.array([])
    user_type_est = []
    used_policy=2 # average_takeOneReassess_loyal #int(self.used_policy)
    #observation_signal=self.observation_vector
    # signal = [O1,O2,O3,O4,O5,O6]
    #num_of_observables=int(num_obs)
    # new signal to update belief
    tCount =humtypes.size
    
    # for each human participant. Dont forget to reset beta and the other variables
    for user_id in range(len(user_obs)):
        current_belief = np.array([]) # probability distribution over (types)
        current_belief = np.ones((humtypes.size))/(humtypes.size)
        beta=current_belief # temp current belief
        belief_list = []
        part_list = []
        task_id = 0
        for i in range(8): # in total each participant did 8 tasks
            task_id += 1
            if task_id != 1 and task_id != 5: # those are the tasks human did the task alone (no robot)
                # collect the obs emitted within a certain task from the participant
                temp_obs_arr = []
                for j in range(len(user_obs[user_id])):
                    if (task_id == taskID_arr[user_id][j]):
                        temp_obs_arr.append(user_obs[user_id][j])
                P_list=np.array([])
                #for used_policy in range(len(observation_model)):
                P=np.array([]) # temporary belief
                for humtype in range(0,tCount):
                    p = 1.0
                    # for each episode
                    for episode in range(0,len(temp_obs_arr)):
                        #observation_number=0
                        # From the observation signal at each episode, a number is generated and probabilities are kept under that number,
                        # [ 1 * 64 ] rows are generated and saved into observation_model.
                        # Now, it is time to use that information.
                        # Calculating the observation number for each episode:
                        #for i in range(observation_signal[0,:].size):
                        #    observation_number = observation_number + (2**i)*observation_signal[episode,i]
                        observation_number=int(temp_obs_arr[episode])
                        if (observation_number == 41 or observation_number == 43 or observation_number == 40):
                            observation_number = 9
                        elif (observation_number == 25):
                            observation_number = 17
                        #elif (observation_number==5):
                        #    continue
                        #elif (observation_number == 33):
                        #    continue
                        # observation_number = observation_to_featureVec(observation_number)
                        # Take the observation row for corresponding (policy, human type) pair.
                        observation_row=observation_model[used_policy,humtype]
                        observation_row=np.array(observation_row)
                        # Calculating the probability (with respect to each observation number) for each episode
                        p = p * (observation_row[observation_number]+1e-4) # 1e-5
                        #print observation_row[observation_number]+1e-4, p       
                    # New belief for this human type
                    # TODO: add an epsilon value below to beta.items. Currently after a time belief never changes
                    P=np.append(P, p*beta.item(humtype))
                #P_list=np.concatenate((P_list, P/(P.sum())), axis=0)
                # update and normalise belief
                #print P_list

               # temp_bel = np.array([])
        
                #for lin in range(tCount):
                #    np.append(temp_bel, np.mean(P_list[:][lin]))
                #    print temp_bel
                #beta = temp_bel
                #print beta
                beta=P/(P.sum())
                print "Part_id:", user_id+4, "Task_id:", task_id, "Used_policy", used_policy, "HUMAN_TYPE:", np.argmax(beta)
                # record beliefs just to observe
                np.append(beliefSet,beta)
                belief_list.append(np.argmax(beta))
        part_list.append(user_id+4)
        part_list.append(belief_list)
        user_type_est.append(part_list)
    
    print user_type_est
            
    '''
    dirr = os.path.dirname(os.path.realpath(__file__))
    with open(dirr + "/abps_savings.json") as file:
        # abps_savings_file= dirr + "/abps_savings.json" # use previously saved belief and observables for the same human interacted
        # abps_savings= open(abps_savings_file).read()
        abps_data = json.load(file)
        abps_data["current_belief"] = self.current_belief.tolist()
        abps_data["beliefSet"] = self.beliefSet.tolist()
    file.close()
    
    with open(dirr + "/abps_savings.json", 'w') as f:
        json.dump(abps_data, f)
    f.close()
    '''
    #each belief array should be recorded, write it to a different file?
    
    #plot the distribution of each participant over each human type

def set_comparisons(mu1, mu2, mu3, mu_f, std1, std2, std3, std_f):
    
    plot_0 = []
    
    for i in range(len(mu1)):
        for j in range(len(mu1[0])):
            if i == 3:
                plt.plot(j, mu1[i][j], color='green', marker='o')
                plt.plot(j, mu2[i][j], color='red', marker='o')
                plt.plot(j, mu3[i][j], color='yellow', marker='o')
                plt.plot(j, mu_f[i][j], color='blue', marker='s')

# def plot_sets(mean1, mean2, mean3, mu_f, std1, std2, std3, std_f):

def remove_reminder_models(mu_f, std_f, obs_f):
    
    # this below decreases the effect of the reminder models
    for i in range(len(mu1)):
        for j in range(len(mu1[0])):
            if (i!=1 and i!=7) and (j==0 or j==1 or j==6 or j==7 or j==12 or j==13):
                mu_f[i][j] = mu_f[i][j] - 20
    # this below directly removes the reminder models, overwriting above
    mu_f = np.delete(mu_f, [0, 1, 6, 7, 12, 13], 1)
    std_f = np.delete(std_f, [0, 1, 6, 7, 12, 13], 1)
    
    # obs_f 18, 8, 64
    '''
    obs_f_new = []
    for j in range(len(obs_f)):
        if (j!=0 and j!=1 and j!=6 and j!=7 and j!=12 and j!=13):
            obs_f_new.append(obs_f[j])
    '''
    obs_f = np.delete(obs_f, [0, 1, 6, 7, 12, 13], 0)
    
    return mu_f, std_f, obs_f          
    

# combinedMean and variance calculation
def combine_mean_variance(mean1, mean2, std1, std2):
    
    x, y = len(mean1), len(mean1[0])
    combinedMean = np.zeros((x, y))
    combinedVar = np.zeros((x, y))
#    combinedVar = std1

    for i in range(len(mean1)):
        for j in range(len(mean1[0])):
            combinedMean[i][j] = (mean1[i][j] + mean2[i][j]) / 2
            if j == 13 and i != 1 and i != 3 and i != 5: # to update reactive robot rewards (wrongly calculated)
                combinedMean[i][j] = combinedMean[i][j] - 2
            d1_square = (mean1[i][j] - combinedMean[i][j]) * (mean1[i][j] - combinedMean[i][j])
            d2_square = (mean2[i][j] - combinedMean[i][j]) * (mean2[i][j] - combinedMean[i][j])
            combinedVar[i][j] = ((std1[i][j] + d1_square) + (std2[i][j] + d2_square)) / 2
        
    return combinedMean, combinedVar


if __name__== "__main__":
    
    dir_path = os.path.dirname(os.path.realpath(__file__))
    json_file= dir_path + "/../../../../../../../configs/scenario_config.json"
    json_data= open(json_file).read()
    data = json.loads(json_data)
    
    dataset1=loadmat("userStudies_1stRun.mat")
    dataset2=loadmat("userStudies_2ndRun.mat")
    dataset3=loadmat("userStudies_3rdRun.mat")
    dataset_f = loadmat("userStudies_final.mat")
    #dataset1_2 = loadmat("exp1-2_1400t.mat")
    
    obs1=dataset1['observation_model']
    obs2=dataset2['observation_model']
    obs3=dataset3['observation_model']
    obs_f=dataset_f['observation_model']
    num_obs=dataset_f['num_of_observables']
    hum_types=dataset_f['humtypes']
    
    mu1=dataset1['mu_model']
    mu2=dataset2['mu_model']
    mu3=dataset3['mu_model']
    mu_f=dataset_f['mu_model']

    std1=dataset1['std_model']
    std2=dataset2['std_model']
    std3=dataset3['std_model']
    std_f=dataset_f['std_model']
    
    policies_new=list(data["evaluation_models"]["policies"])
    
    [mu_f_new, std_f_new, obs_f_new]= remove_reminder_models(mu_f, std_f, obs_f)
    savemat("userStudies_final_v2.mat",{'policies':policies_new, 'humtypes':dataset_f["humtypes"],
                                        'mu_model':mu_f_new,'std_model':std_f_new, 'observation_model':obs_f_new,
                                        'num_of_observables':num_obs})

    # bayesian_estimation(obs_f, num_obs, hum_types)
    # mu1_2, std1_2 = combine_mean_variance(mu1, mu2, std1, std2)
        
    # obs1_2 = (obs1 + obs2) / 2
#    
#    dataset = dataset1
#    dataset['mu_model'] = mu1_2
#    dataset['std_model'] = std1_2
#    dataset['observation_model'] = obs1_2
#    savemat("exp1-2_1400t.mat", dataset)
#    
    # REMOVE BELOW AFTER MODIFICATIONS
    #dataset1['observation_model'] = obs1
    #dataset2['observation_model'] = obs2
    #savemat("exp1_1400t.mat", dataset1)
    #savemat("exp2_1400t.mat", dataset2)

    """
    s=[]
    polavg=[]
    human_avg_obs=[]
    plt.figure()
    for k in range(0,5):
        s.append(obs1_2[k,:])
        
    s=np.array(s)
    for a in range(0,10):
        for b in range(0,64):
            polavg.append(np.sum(s[:,a,b]))
        human_avg_obs.append(polavg)
        polavg=[]
        
    human_avg_obs=np.array(human_avg_obs)
    for i in range(0,9):
        if i==7 or i == 0 :
            plt.plot(range(0,64),human_avg_obs[i,:])
    
    plt.legend(( "distracted_collaborative.POMDPx",                    
                         "beginner_nontired_noncollaborative.POMDPx"),loc=1)
    
    plt.grid()
    
    plt.annotate('successful', xy=(5, 0.06), xytext=(5, 0.06),
                arrowprops=dict(facecolor='black', shrink=0.005),
                )
    
    plt.annotate('walk away', xy=(0, 0.05), xytext=(0, 0.05),
                arrowprops=dict(facecolor='black', shrink=0.005),
                )
    
    plt.annotate('lookaround&idle', xy=(35, 0.02), xytext=(35, 0.02),
                arrowprops=dict(facecolor='black', shrink=0.005),
                )
    plt.annotate('idle', xy=(33, 0), xytext=(33, 0),
                arrowprops=dict(facecolor='black', shrink=0.005),
                )
    plt.annotate('warn', xy=(17, 0), xytext=(17, 0),
                arrowprops=dict(facecolor='black', shrink=0.005),
                )
    plt.annotate('fail', xy=(9, 0), xytext=(9, 0),
                arrowprops=dict(facecolor='black', shrink=0.005),
                )
    
    plt.title("Average Observations")
    """
