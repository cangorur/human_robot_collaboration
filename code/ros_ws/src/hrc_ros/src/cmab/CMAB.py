#!/usr/bin/env python3

from hrc_ros.srv import *
from hrc_ros.msg import *
import rospy
import numpy as np
from collections import deque
import std_srvs.srv
import BayesianExpert
import pickle
import std_srvs.srv
import SVM
import json


class CMAB():

	def __init__(self):
		bytes_read = open("humanDict.bin", "rb").read()
		self.humanDict = pickle.loads(bytes_read)
		bytes_read = open("humanDictInv.bin", "rb").read()
		self.humanDictInv = pickle.loads(bytes_read)

		bytes_read = open("robotDict.bin", "rb").read()
		self.robotDict = pickle.loads(bytes_read)
		bytes_read = open("robotDictInv.bin", "rb").read()
		self.robotDictInv = pickle.loads(bytes_read)
		bytes_read = open("lut.bin", "rb").read()
		self.lookUpTable = pickle.loads(bytes_read)


		## Last 10 observations used as context
		self.context = deque([np.zeros(6)]*10)
		self.observations = np.zeros(6)
		self.n = 0.0
		self.r = 0.0

		self.Q = np.ones((1,2),dtype=float)/2.0
		self.P = np.ones(11,dtype=float)/11.0
		self.E = self.P
		self.gamma = 0.01
		self.mu=1.0
		self.a_t = 0
		rospy.Service('/cmab/select_policy', SelectPolicy, self.choosePolicy)
		rospy.Subscriber("/task_manager/task_status", TaskState, self.task_status_callback)


	def task_status_callback(self,TaskState):
		"""
		Callback function:
			1. Store observations and rewards to classify the human type
			2. Update trust in experts
		@param TaskState The message published by the /task_manager/task_status
		"""

		observations= self.observations
		r = self.r
		a_t = self.a_t
		n = self.n
		context = self.context

		if TaskState.total_disc_reward != "":
			r = float(TaskState.total_disc_reward)

		if TaskState.task_status =="START":
			if n<1:
				context.append(np.zeros(6))
			else:
				context.append(observations/n)


			context.popleft()
			observations = np.zeros(6)
			self.updateQ(r,a_t)
			n=0.0
		obs = list(TaskState.human_observables)
		l = len(obs)
		if l==0:
			return

		self.observations = observations + np.array(obs)
		self.n = n + 1.


	def choosePolicy(self,x):
		"""
		Service Callback function:
			1. Get experts advice
			2. Sample from human estimate distribution
			3. Choose best policy according to lookup table
		@param x the parameter is not used, just needed for ros service syntax
		"""

		Q = self.Q
		observations = self.observations
		context = self.context

		rospy.loginfo("CMAB")
		obs = np.mean(context,axis=0)
		E1 = SVM.classify(obs.reshape(1,6))
		rospy.loginfo(E1)
		E2 = BayesianExpert.classify(obs)
		self.E = np.array([E1,E2])
		self.P = Q.dot(self.E)[0]
		self.a_t = np.random.choice(list(range(11)),p=self.P)
		robot = self.lookUpTable[self.a_t]
		robot = self.robotDictInv[robot][1:-1]
		return(SelectPolicyResponse(robot))

	def updateQ(self,r,a_t):
		"""
		Based on the received reward and selected action (robot policy)
		update trust in experts by changing the distribution "Q"
		@param r accumulated reward from last task
		@param a_t estimated human type
		"""

		global Q,gamma,P,E
		Q = self.Q
		P = self.P
		E = self.E
		gamma = self.gamma

		r = max(min(6.0,r),-6.0)
		x_t = (r+6.0)/12.0
		X = np.ones(11,dtype=float)/1.0
		X[a_t] = 1.0 - (1.0/(P[a_t]+gamma))*(1.0-x_t)

		X = E.dot(X)
		Q = np.exp(X*self.mu)*Q[0]
		Q = np.reshape(Q/Q.sum(),(1,len(Q)))

if __name__ == "__main__":

	while not rospy.has_param('/cmab_flag'):
		continue
	useCMAB = rospy.get_param('/cmab_flag')
	if useCMAB:
		rospy.init_node("cmab")
		cmab = CMAB()
		rospy.loginfo("CMAB Agent is ready!")
		rospy.spin()
	#rospy.set_param("/human_type", "humanModel_expert.POMDPx")
	#rospy.set_param("/robot_model", "proactive_robot_pomdp.pomdpx")
