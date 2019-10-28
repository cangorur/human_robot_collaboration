#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 26 17:20:57 2019

@author: cangorur
"""

#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 15 14:36:19 2018

@author: dell
"""
from scipy.io import loadmat, savemat
import numpy as np
import matplotlib.pyplot as plt   



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

def remove_reminder_models(mu1, mu2, mu3, mu_f, std1, std2, std3, std_f):
    
    for i in range(len(mu1)):
        for j in range(len(mu1[0])):
            if (i!=1 and i!=7) and (j==0 or j==1 or j==6 or j==7 or j==12 or j==13):
                mu_f[i][j] = mu_f[i][j] - 20


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
    
    dataset1=loadmat("userStudies_1stRun.mat")
    dataset2=loadmat("userStudies_2ndRun.mat")
    dataset3=loadmat("userStudies_3rdRun.mat")
    dataset_f = loadmat("userStudies_final.mat")
    #dataset1_2 = loadmat("exp1-2_1400t.mat")
    
    obs1=dataset1['observation_model']
    obs2=dataset2['observation_model']
    obs3=dataset3['observation_model']
    obs_f=dataset_f['observation_model']
    
    mu1=dataset1['mu_model']
    mu2=dataset2['mu_model']
    mu3=dataset3['mu_model']
    mu_f=dataset_f['mu_model']

    std1=dataset1['std_model']
    std2=dataset2['std_model']
    std3=dataset3['std_model']
    std_f=dataset_f['std_model']
    
    remove_reminder_models(mu1, mu2, mu3, mu_f, std1, std2, std3, std_f)
    
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
