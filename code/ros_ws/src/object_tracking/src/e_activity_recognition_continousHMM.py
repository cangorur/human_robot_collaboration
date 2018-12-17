#!/usr/bin/env python
# Work of Feza Turgay CELIK, Aug 2016

import scipy as sp
from scipy import stats
import numpy as np
import csv
import rospy
import os
from std_msgs.msg import String
from object_tracking.msg import Subfeatures


class gmmhmm:
    #This class converted with modifications from https://code.google.com/p/hmm-speech-recognition/source/browse/Word.m
    def __init__(self, n_states):
	
        self.n_states = n_states
        self.random_state = np.random.RandomState(0)
        
        #Normalize random initial state
        self.prior = self._normalize(self.random_state.rand(self.n_states, 1))
        self.A = self._stochasticize(self.random_state.rand(self.n_states, self.n_states))
        
        self.mu = None
        self.covs = None
        self.n_dims = None
        
# This gmm_hmm code used the forward-backward algorithm to calculate occurance probabilities of each hidden state        
           
    def _forward(self, B):
        
        log_likelihood = 0.
        T = B.shape[1]
        alpha = np.zeros(B.shape)
        
        for t in range(T):
            if t == 0:
                alpha[:, t] = B[:, t] * self.prior.ravel()
            else:
                alpha[:, t] = B[:, t] * np.dot(self.A.T, alpha[:, t - 1])
            
            alpha_sum = np.sum(alpha[:, t])
            alpha[:, t] /= alpha_sum
            log_likelihood = log_likelihood + np.log(alpha_sum)
        return log_likelihood, alpha
        #print (alpha)
    def _backward(self, B):
        T = B.shape[1]
        beta = np.zeros(B.shape);
           
        beta[:, -1] = np.ones(B.shape[0])
            
        for t in range(T - 1)[::-1]:
            beta[:, t] = np.dot(self.A, (B[:, t + 1] * beta[:, t + 1]))
            beta[:, t] /= np.sum(beta[:, t])
        return beta
    
    def _state_likelihood(self, obs):
	
        obs = np.atleast_2d(obs) 
        B = np.zeros((self.n_states, obs.shape[1]))
        for s in range(self.n_states):
            
            #Needs scipy 0.14
            B[s, :] = sp.stats.multivariate_normal.pdf(obs.T, mean=self.mu[:, s].T, cov=self.covs[:, :, s].T)
        
            #This function can (and will!) return values >> 1
            #See the discussion here for the equivalent matlab function
            #https://groups.google.com/forum/#!topic/comp.soft-sys.matlab/YksWK0T74Ak
            #Key line: "Probabilities have to be less than 1,
            #Densities can be anything, even infinite (at individual points)."
            #This is evaluating the density at individual points...
        return B
    
    def _normalize(self, x):
        return (x + (x == 0)) / np.sum(x)
    
    def _stochasticize(self, x):
        return (x + (x == 0)) / np.sum(x, axis=1)
    
    def _em_init(self, obs):
        #Using this _em_init function allows for less required constructor args
        if self.n_dims is None:
            self.n_dims = obs.shape[0]
        if self.mu is None:
            subset = self.random_state.choice(np.arange(self.n_dims), size=self.n_states, replace=False)
            self.mu = obs[:, subset]
        if self.covs is None:
            self.covs = np.zeros((self.n_dims, self.n_dims, self.n_states))
            self.covs += np.diag(np.diag(np.cov(obs)))[:, :, None]
        return self
    
    def _em_step(self, obs): 
        obs = np.atleast_2d(obs)
        B = self._state_likelihood(obs)
        T = obs.shape[1]  #frame sayisi
        
        log_likelihood, alpha = self._forward(B)
        beta = self._backward(B)
        
        
        xi_sum = np.zeros((self.n_states, self.n_states)) #nxn
        gamma = np.zeros((self.n_states, T))
        
        for t in range(T - 1):
            partial_sum = self.A * np.dot(alpha[:, t], (beta[:, t] * B[:, t + 1]).T)
            xi_sum += self._normalize(partial_sum)
            partial_g = alpha[:, t] * beta[:, t]
            gamma[:, t] = self._normalize(partial_g)
              
        partial_g = alpha[:, -1] * beta[:, -1]
        gamma[:, -1] = self._normalize(partial_g)
        
        expected_prior = gamma[:, 0]
        expected_A = self._stochasticize(xi_sum)
        
        expected_mu = np.zeros((self.n_dims, self.n_states))
        expected_covs = np.zeros((self.n_dims, self.n_dims, self.n_states))
        
        gamma_state_sum = np.sum(gamma, axis=1)
        #Set zeros to 1 before dividing
        gamma_state_sum = gamma_state_sum + (gamma_state_sum == 0)
        
        for s in range(self.n_states):
            gamma_obs = obs * gamma[s, :]
            expected_mu[:, s] = np.sum(gamma_obs, axis=1) / gamma_state_sum[s]
            partial_covs = np.dot(gamma_obs, obs.T) / gamma_state_sum[s] - np.dot(expected_mu[:, s], expected_mu[:, s].T)
            #Symmetrize
            partial_covs = np.triu(partial_covs) + np.triu(partial_covs).T - np.diag(partial_covs)
        
        #Ensure positive semidefinite by adding diagonal loading
        expected_covs += .01 * np.eye(self.n_dims)[:, :, None]
        
        self.prior = expected_prior
        self.mu = expected_mu
        self.covs = expected_covs
        self.A = expected_A
        return log_likelihood
    
    def fit(self, obs, n_iter=15):
        #Support for 2D and 3D arrays
        #2D should be n_features, n_dims
        #3D should be n_examples, n_features, n_dims
        #For example, with 6 features per speech segment, 105 different words
        #this array should be size
        #(105, 6, X) where X is the number of frames with features extracted
        #For a single example file, the array should be size (6, X)
        if len(obs.shape) == 2:
            for i in range(n_iter):
                self._em_init(obs)
                log_likelihood = self._em_step(obs)
        elif len(obs.shape) == 3:
            count = obs.shape[0]
            for n in range(count):
                for i in range(n_iter):
                    self._em_init(obs[n, :, :])
                    log_likelihood = self._em_step(obs[n, :, :])
          
        elif len(obs.shape) == 100:
            count = obs.shape[0]
            for n in range(count):
                for i in range(n_iter):
                    self._em_init(obs[n, :, :])
                    log_likelihood = self._em_step(obs[n, :, :])
        return self
    
    def transform(self, obs):
        #Support for 2D and 3D arrays
        #2D should be n_features, n_dims
        #3D should be n_examples, n_features, n_dims
        #For example, with 6 features per speech segment, 105 different words
        #this array should be size
        #(105, 6, X) where X is the number of frames with features extracted
        #For a single example file, the array should be size (6, X)
        if len(obs.shape) == 2:
            B = self._state_likelihood(obs)
            log_likelihood, _ = self._forward(B)
            return log_likelihood
        elif len(obs.shape) == 3:
            count = obs.shape[0]
            out = np.zeros((count,))
            for n in range(count):
                B = self._state_likelihood(obs[n, :, :])
                log_likelihood, _ = self._forward(B)
                out[n] = log_likelihood
            return out
	elif len(obs.shape) == 100:
	    count = obs.shape[0]
            out = np.zeros((count,))
            for n in range(count):
                B = self._state_likelihood(obs[n, :, :])
                log_likelihood, _ = self._forward(B)
                out[n] = log_likelihood
            return out

#This part of the code is used for training.I pool the data separately for every training person.Also i have pool data separately according to the action type. For example, Can's and Feza's data is pooled separately, also Can's data is pooled in four different segments such as Can_Sitting , Can_Standing, Can_Walking, Can_Leaning_Forward.(By this way i didn't use the labelling data)  

def gmm_model():
	dir_path = os.path.dirname(os.path.realpath(__file__))
	dataset_dir = dir_path + "/../../../datasets"
    
	#read Minh data
	path1 = dataset_dir + '/DataSet_Minh/idleSet.csv'
	with open(path1, 'rb') as f:
	    reader = csv.reader(f)
	    data_as_list = list(reader)
	M_idle = [[float(column) for column in row] for row in data_as_list]

	path2 = dataset_dir + '/DataSet_Minh/graspingSet.csv'
	with open(path2, 'rb') as f:
	    reader = csv.reader(f)
	    data_as_list = list(reader)
	M_grasp = [[float(column) for column in row] for row in data_as_list]

	path3 = dataset_dir + '/DataSet_Minh/stopGestureSet.csv'
	with open(path3, 'rb') as f:
	    reader = csv.reader(f)
	    data_as_list = list(reader)
	M_gesture = [[float(column) for column in row] for row in data_as_list]

	path4 = dataset_dir + '/DataSet_Minh/undefinedSet.csv'
	with open(path4, 'rb') as f:
	    reader = csv.reader(f)
	    data_as_list = list(reader)
	M_undefined = [[float(column) for column in row] for row in data_as_list]

	#combining train set
	T_Idle1= M_idle #+ Feza_LF + Can_LF
	T_Grasping1= M_grasp #+ Feza_sit + Can_sit
	T_Gesture1= M_gesture #+ Feza_stand + Can_stand
	T_Undefined1= M_undefined #+ Feza_stand + Can_stand


	T_Idle= np.array(T_Idle1)
	T_Grasping= np.array(T_Grasping1)
	T_Gesture= np.array(T_Gesture1)
	T_Undefined= np.array(T_Undefined1)


	T_Idle_T= T_Idle.transpose(1,0)
	T_Grasping_T= T_Grasping.transpose(1,0)
	T_Gesture_T= T_Gesture.transpose(1,0)
	T_Undefined_T= T_Undefined.transpose(1,0)

# excluding labels of the data, since the separation of data for clusters are done manually
	T_Idle_All= T_Idle_T [:2,:500] 
	T_Grasping_All= T_Grasping_T [:2,:500]  
	T_Gesture_All= T_Gesture_T [:2,:500] 
	T_Undefined_All= T_Undefined_T [:2,:500] 
 

	train1 = np.ones((2, 500)) + .1 *T_Idle_All
	train1 /= train1.sum(axis=0)
	    
	train2 = np.ones((2,500)) + .1 *T_Grasping_All
	train2 /= train2.sum(axis=0)
	
	train3 = np.ones((2,500)) + .1 *T_Gesture_All
	train3 /= train3.sum(axis=0)
	
	train4 = np.ones((2,500)) + .1 *T_Undefined_All
	train4 /= train4.sum(axis=0)

	
	#trainset fitting 
	m1 = gmmhmm(2)
	m1.fit(train1)
	m2 = gmmhmm(2)
	m2.fit(train2)
	m3 = gmmhmm(2)
	m3.fit(train3) 
	m4 = gmmhmm(2)
	m4.fit(train4) 
	print "Modeling is succesfull ! "
	return m1, m2, m3, m4


def callback(data):
	temp = [0,0]
	temp[0] = float(data.velocity)
	temp[1] = float(data.distance)
	feature_vector = temp
	print feature_vector
	data_observation.append(feature_vector)
	data_obs = np.asarray(data_observation)
	if len(data_observation) > 9 :
		data_obs_T = data_obs.transpose(1,0)
		test = np.ones((2,10)) + .1 *data_obs_T
		test /= test.sum(axis=0)
		m1t1 = m1.transform(test)
		m2t2 = m2.transform(test)
		m3t3 = m3.transform(test)
		m4t4 = m4.transform(test)
			

		print ("Likelihoods for test set ")
		print ("Idle(1):",m1t1)
		print ("Grasping(2):",m2t2)
		print ("Gesture(3):",m3t3)
		print ("Undefined(4):",m4t4)
		print ("Prediction for test set ")
		print ("Model", np.argmax([m1t1, m2t2, m3t3, m4t4]) + 1)
		del data_observation[0]

 	

def run():
	global feature_vector
	global single_vect
	global m1
	global m2
	global m3
	global m4
	global data_observation

	feature_vector = []
	rospy.init_node('activitiy_recognition_continousHMM_node',anonymous=True)
	obsFreq = 100
	rate=rospy.Rate(obsFreq)
	## call model constructor once
	m1, m2, m3, m4 = gmm_model()	
	
	data_observation = []
	rospy.Subscriber("/camera_agent/motion_features",Subfeatures,callback)

	rospy.spin()


if __name__ == "__main__":
    
    try:
	run()
    except rospy.ROSInterruptException:
	pass
dataFile.close()   
    
   
