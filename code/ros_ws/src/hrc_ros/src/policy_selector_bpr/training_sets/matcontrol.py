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

def estimate(obs_vector, observation_model, current_belief, used_policy):

    beta=current_belief # temp current belief
    # record beliefs just to observe

    used_policy=int(self.used_policy)
    observation_signal=self.observation_vector
    # signal = [O1,O2,O3,O4,O5,O6]
    num_of_observables=int(self.num_of_observables)
    humtypes=self.humtypes
    # new signal to update belief
    tCount =humtypes.size
    P=np.array([]) # temporary belief
    # for each human Type
    for humtype in range(0,tCount):
        p = 1.0
        # for each episode
        for episode in range(0,int(observation_signal.size/num_of_observables)):
            observation_number=0
            # From the observation signal at each episode, a number is generated and probabilities are kept under that number,
            # [ 1 * 64 ] rows are generated and saved into observation_model.
            # Now, it is time to use that information.
            # Calculating the observation number for each episode:
            for i in range(observation_signal[0,:].size):
                observation_number = observation_number + (2**i)*observation_signal[episode,i]
            observation_number=int(observation_number)
            # observation_number = observation_to_featureVec(observation_number)
            # Take the observation row for corresponding (policy, human type) pair.
            observation_row=observation_model[used_policy,humtype]
            observation_row=np.array(observation_row)
            # Calculating the probability (with respect to each observation number) for each episode
            p = p * (observation_row[observation_number]+1e-4) # 1e-5
        # New belief for this human type
        # TODO: add an epsilon value below to beta.items. Currently after a time belief never changes
        P=np.append(P, p*beta.item(humtype))

    # update and normalise belief
    beta=P/(P.sum())
    rospy.logwarn("[BPR_POLICY_SELECTOR] -------Belief is Updated-----")
    rospy.logwarn(beta)
    rospy.logwarn("[BPR_POLICY_SELECTOR] Most likely human type: %d", np.argmax(beta))
    rospy.logwarn("-----------------------------")
    time.sleep(2)
    self.current_belief=np.array(beta)

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


def combine_mean_variance_for3(mean1, mean2, mean3, std1, std2, std3):

    x, y = len(mean1), len(mean1[0])
    combinedMean = np.zeros((x, y))
    combinedVar = np.zeros((x, y))
#    combinedVar = std1

    for i in range(len(mean1)):
        for j in range(len(mean1[0])):
            combinedMean[i][j] = (mean1[i][j] + mean2[i][j] + mean3[i][j]) / 3
            d1_square = (mean1[i][j] - combinedMean[i][j]) * (mean1[i][j] - combinedMean[i][j])
            d2_square = (mean2[i][j] - combinedMean[i][j]) * (mean2[i][j] - combinedMean[i][j])
            d3_square = (mean3[i][j] - combinedMean[i][j]) * (mean3[i][j] - combinedMean[i][j])
            combinedVar[i][j] = ((std1[i][j] + d1_square) + (std2[i][j] + d2_square) + (std3[i][j] + d3_square)) / 3

    return combinedMean, combinedVar

if __name__== "__main__":

    dataset1=loadmat("./userStudies/userStudies_1stRun.mat")
    dataset2=loadmat("./userStudies/userStudies_2ndRun.mat")
    dataset3=loadmat("./userStudies/userStudies_3rdRun.mat")

    obs1=dataset1['observation_model']
    obs2=dataset2['observation_model']
    obs3=dataset3['observation_model']
    #obs1_2=dataset1_2['observation_model']

    mu1=dataset1['mu_model']
    mu2=dataset2['mu_model']
    mu3=dataset3['mu_model']
    #mu1_2=dataset1_2['mu_model']

    std1=dataset1['std_model']
    std2=dataset2['std_model']
    std3=dataset3['std_model']
    #std1_2=dataset1_2['std_model']

    #mu1_2, std1_2 = combine_mean_variance(mu1, mu2, std1, std2)

    mu1_2_3, std1_2_3 = combine_mean_variance_for3(mu1, mu2, mu3, std1, std2, std3)
    obs1_2_3 = (obs1 + obs2 + obs3) / 3
#
    dataset = dataset1
    dataset['mu_model'] = mu1_2_3
    dataset['std_model'] = std1_2_3
    dataset['observation_model'] = obs1_2_3
    savemat("userStudies_final.mat", dataset)
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
