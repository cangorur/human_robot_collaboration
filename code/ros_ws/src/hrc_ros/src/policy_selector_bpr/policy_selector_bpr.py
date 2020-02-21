#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  6 12:05:55 2018
@author: gunerdilsader
"""
import numpy as np
from scipy.io import loadmat  #the SciPy module that loads and saves mat-files
import math
import rospy
from hrc_ros.srv import PolicySelectorBPR, PolicySelectorBPRResponse
from std_msgs.msg import String
from std_msgs.msg import Int64
from hrc_ros.msg import TaskStateIE
import json
import os
import time

class PolicySelector:
    '''
        This class chooses a human type and offers best fitting policy
    '''
    def __init__(self, data):
        self.file = data

        self.humtypes=np.array([])
        self.policies=np.array([])
        self.policy_list_json=np.array([])

        self.isNewHuman = rospy.get_param('/is_new_human')
        self.isFirstRun = True

        self.observation_vector=np.array([]) # observations
        self.observation_signal= np.array([])
        self.reward = np.array([])
        self.last_reward_signal= np.array([])
        self.current_policy = -1
        self.used_policy = -1
        # TODO: all these variables below need to be taken from json file
        self.current_belief = np.array([]) # probability distribution over (types)
        self.prev_belief = np.array([])
        self.beliefSet= np.array([])
        self.taken_policies_set= np.array([])

        # self.contInteraction = pref # by default in user studies there is no continous interaction due to the experiments
        self.observation_model= np.array([]) # (types, policies)

        self.selected_policy=np.array([])# important output

        self.select_policy_service= rospy.Service('/policy_selector_bpr/select_policy',PolicySelectorBPR, self.run_policy_selector)

        rospy.Subscriber("/task_manager/task_status", TaskStateIE, self.observation_update)
        # we will not subcribe for reward seperately, will be under observation_update
        self.initialize_models() # Once class is called, models are initialized automatically.

    def initialize_models(self):
        '''
            This function creates a uniform probability distribution over human types as initial belief
                          and runs the functions that import observation model and performance model from training set.
        '''
        self.performance_model_constructor(self.file)
        self.observation_model_constructor(self.file)

    def run_policy_selector(self, req):
        '''
            This function is the callback function of BPRPolicySelector (\select_policy) service
                            runs the functions that select best policy and that updates the current belief
                @param request
                @return self.selected_policy
        '''
        dirr = os.path.dirname(os.path.realpath(__file__))
        if self.isNewHuman:
            abps_savings_file= dirr + "/abps_savings_new.json" # variables are all zeros, newly initialized
        else:
            abps_savings_file= dirr + "/abps_savings.json" # use previously saved belief and observables for the same human interacted

        abps_savings= open(abps_savings_file).read()
        abps_data = json.loads(abps_savings)

        # TODO: all these variables below need to be taken from json file
        self.observation_vector=np.array(abps_data["observation_vector"]) # observations
        self.observation_signal= abps_data["observation_signal"] #[]
        self.reward= np.array(abps_data["reward"]) # np.array([]) # utility
        self.last_reward_signal= np.array(abps_data["last_reward_signal"]) #np.array([])
        self.current_policy = abps_data["current_policy"]#-1
        self.used_policy = abps_data["used_policy"]#-1
        # TODO: all these variables below need to be taken from json file
        self.current_belief = np.array([]) # probability distribution over (types)
        self.prev_belief = np.array(abps_data["current_belief"])
        self.beliefSet= np.array(abps_data["beliefSet"]) # just to observe past
        self.taken_policies_set= np.array(abps_data["taken_policies_set"]) # just to observe past

        # if running for a new user  ! ! !
        print ("PREV BELIEF", self.prev_belief)
        if self.isNewHuman or self.prev_belief == []:
            self.current_belief = np.ones((self.humtypes.size))/(self.humtypes.size)
            self.isNewHuman = False

        else:
            self.current_belief = self.prev_belief

        self.belief_updater()
        self.policy_selector()
        rospy.logwarn("[BPR_POLICY_SELECTOR] Selected Policy: %d", self.selected_policy)
        return(PolicySelectorBPRResponse(self.selected_policy, self.current_belief))

    def performance_model_constructor(self,file):
        '''
            This function imports performance models from training set.
                @param file name (training set .mat file)
        '''
        # performance models: mu_model and std_model
        # mu_model  : average of reward values belongs to each (type,policy) pair
        # std_model : standard deviations of reward values belongs to each (type,policy) pair

        dataset = loadmat(file)

        self.std_model= dataset['std_model']
        self.mu_model = dataset['mu_model']
        self.policies = dataset['policies']  # variable in mat file
        self.humtypes = dataset['humtypes']  # variable in mat file

    def observation_model_constructor(self,file):
        '''
            This function imports observation model and number of observables from training set.
                @param file name (training set .mat file)
        '''
        # observation_model : disributions(observation, types, policies)
        dataset = loadmat(file)
        self.observation_model= dataset['observation_model'] # variable in mat file
        self.num_of_observables=dataset['num_of_observables'] # variable in mat file

    def observation_update(self, data):
        '''
            This function is the callback function of the task_manager/task_status subscriber.
                Observation vector and total discounted reward values are published in task_manager/task_status topic.
            This function keeps the last total discounted reward and all the observation vectors during a task,
                then, when new task starts, ('MANAGER','START')
                observation vector and total discounted reward are propagated to self.observation_vector and self.reward
                to select best policy.

                @param: task_manager/task_status topic messages
                @TODO: self.policy_list_json should be identical with the self.policies.
                (TO detect which policy is used in previous task, there should be a list of policies.
                However, policy list imported from .mat file is not suitable to use.
                Policy names were filled with spaced to equate the length of each policy name.
                We need to have the names without spaces. I imported them from the json file.
                So, order of the policies and names should remain the same after building training set.
                Or a solution should be found in order to have the policy names without spaces.)
                @TODO: Can: I believe this is not needed. We already know which policy selected in the new format.
                Policies have IDs that can be matched directly.
        '''

        if (data.who_reports == 'MANAGER' and data.task_status == 'START'):

            rospy.loginfo("New task started")


            # Initialize observation and reward signal to be filled again in the next task
            self.observation_signal=[]
            self.last_reward_signal=np.array([])
            self.current_policy=-1

        if (data.who_reports == 'OBSERVATION'):

            instant_observation_str=data.human_observables
            instant_observation=np.zeros(len(data.human_observables))
            for i in range(0,len(data.human_observables)):
                instant_observation[i]=int(bool(instant_observation_str[i]))

            self.observation_signal.append(instant_observation)

    #def reward_update(self,instant_reward):
        #reward comes from another ros agent, make it useful
        if (data.who_reports == 'ROBOT'):
            self.last_reward_signal=data.total_disc_reward

        if (data.who_reports == 'ROBOT' and ( data.real_state == 'GlobalSuccess' or data.real_state == 'GlobalFail')):
            # If task is finished; propagate observation and reward
            # Then initilize them.
            rospy.loginfo("Task is finished")
            self.observation_vector=np.array(self.observation_signal)
            [[self.current_policy]]=np.where(self.policy_list_json==data.robot_model)
            self.used_policy=self.current_policy
            self.reward=self.last_reward_signal
            rospy.loginfo("[BPR_POLICY_SELECTOR]-----Task is finished-----")
            rospy.loginfo("[BPR_POLICY_SELECTOR] task id: %d,reward: %s, used policy: %d", data.task_id-1, self.reward, self.used_policy)
            rospy.loginfo("[BPR_POLICY_SELECTOR]---------------------------------")

            dirr = os.path.dirname(os.path.realpath(__file__))
            abps_savings_file= dirr + "/abps_savings.json" # use previously saved belief and observables for the same human interacted
            abps_savings= open(abps_savings_file).read()
            abps_data = json.loads(abps_savings)

            abps_data["observation_vector"] = self.observation_vector.tolist()
            abps_data["current_policy"] = self.current_policy
            abps_data["used_policy"] = self.used_policy
            abps_data["reward"] = self.reward
            abps_data["taken_policies_set"] = self.taken_policies_set.tolist()

            with open(dirr + "/abps_savings.json", 'w') as f:
                json.dump(abps_data, f)
            f.close()

            # To find out which policy is used during this task;
            # This is an important information for belief update

    def policy_selector(self):
        '''
            This function selects the best fitting policy using current belief.
        '''

        beta=self.current_belief

        """ BPR - Beta Sampling
        # approximate expected performance based on means
        # select based on sampling beta!!!! BPR-Beta Sampling
        # v = sum(cumsum(belief) < np.random.rand(1)) + 1
        v=0
        Rand=np.random.rand(1)
        Cum=np.cumsum(beta)
        for j in range(0,beta.size):
            if (Cum.item(j) < Rand):
                 v = v + 1
        [tbeta] = np.zeros((1,beta.size));
        tbeta[v] = 1
        exptU = np.matmul([tbeta],self.mu_model)
        maxPi = np.argmax(exptU + np.random.rand(1,exptU.size)*1e-5);
        """
        ## BPR EI ##
        # Compute EI
        vEI=np.array(self.computeEI(beta,self.mu_model,self.std_model) )
        self.vEI=vEI
        # choose best policy
        maxPiEI = np.argmax(vEI + np.random.rand(1,vEI.size)*1e-5)
        # return to selected policy
        self.selected_policy=maxPiEI
        if self.isFirstRun:
            self.selected_policy = -1
            self.isFirstRun = False
        # record taken policies just to observe
        self.taken_policies_set=np.append(self.taken_policies_set,self.selected_policy)

     ###############Expected Improvement Implementation#####################
    # compute EI
    def computeEI(self, beta, mus, sigs):
        '''
        This function computes EI and builds vEI.
        @TODO: Double check the math !
        '''
        beta=self.current_belief
        theta_pi_t = np.zeros(self.policies.size)
        vEI = np.zeros(self.policies.size)
        # for each policy
        for pol in range(0,self.policies.size):
            # expected reward of policy on all types
            expctdS = (mus[:,pol]).transpose()
            # expected reward of policy on belief
            theta_pi_t[pol] = np.matmul(beta,expctdS)
        Ubeta = np.max(theta_pi_t) # max expected utility from all policies
        linspace=np.array(range(0,600,6))
        Uplus = Ubeta + linspace*(np.max(mus)+np.max(sigs)-Ubeta)/600
        # compute online PI
        for pol in range(0,self.policies.size):
            [temp] = self.evaluateFSample(mus[:,pol], sigs[:,pol]+2, Uplus)
            #dilsad: I do not understand the actual reason behind "+2"
            beta=np.array(beta)
            vEI[pol] = np.sum(np.matmul(temp,beta))
        return[vEI]

    # evaluate Gaussian F at positions s
    def evaluateFSample(self,mu,sig,s):
        sig = np.tile(sig, (len(s),1))
        mu = np.tile(mu, (len(s),1))
        #print(np.shape(mu))
        if not(np.size(s) == np.size(mu)):
            s = np.tile(s, (self.humtypes.size,1))
        #error
        #print(np.shape(sig),np.shape(mu),np.shape(s))
        d = s.transpose() - mu
        #print(d.shape)
        # Gaussian modelling of f
        F = 1/(math.sqrt(2*math.pi)*sig)*np.exp(-0.5*np.power(d,2)/(np.power(sig,2)))
        return[F]

    def belief_updater(self):
        '''
            This function updates belief over human types.
        '''
        #using observation information and model
        observation_model=self.observation_model
        beta=self.current_belief # temp current belief
        # record beliefs just to observe
        np.append(self.beliefSet,beta)

        if (self.used_policy == None):
            return

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
                # below observation number updates is for real setup. Mapping some obs to the obs obtained from simulation
                if (observation_number==40 or observation_number==41 or observation_number==43):
                    observation_number = 9
                elif (observation_number == 25):
                    observation_number = 17
                # observation_number = observation_to_featureVec(observation_number)
                # Take the observation row for corresponding (policy, human type) pair.
                #observation_row=observation_model[used_policy,humtype]
                observation_row=observation_model[humtype]
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

    def observation_to_featureVec(obs_digit):
        # TODO manual mapping of 64 bit digit to the feature vectors
        if obs_digit == 1:
            return 1
        elif obs_digit == 2:
            return 2

if __name__=='__main__':

    while not rospy.has_param('/bpr_flag'):
        continue
    useBPR = rospy.get_param('/bpr_flag')
    if useBPR:
        rospy.init_node('policy_selector_bpr', anonymous=True)
        dirr = os.getcwd()
        json_file= dirr + "/../../../../../configs/scenario_config.json"
        json_data= open(json_file).read()
        data = json.loads(json_data)
        file = data["operation_modes"]["trainingSetForPolicySelect"]
        pref = data["operation_modes"]["continousInteraction"] #not used for now
        policy_selector_class = PolicySelector(file)
        policy_selector_class.policy_list_json=np.array(list(data["evaluation_models"]["policies"]))
        rospy.loginfo("[BPR_POLICY_SELECTOR] BPR Policy Selector agent is ready!")
        rospy.spin()

    '''
    @TODO: TO detect which policy is used in previous task, there should be a list of policies.
    However, policy list imported from .mat file is not suitable to use.
    Policy names were filled with spaced to equate the length of each policy name.
    We need to have the names without spaces. I imported them from the json file.
    So, order of the policies and names should remain the same after building training set.
    Or a solution should be found in order to have the policy names without spaces.
    '''
