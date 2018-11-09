#!/usr/bin/env python

from hrc_ros.srv import *
from hrc_ros.msg import *
from std_srvs.srv import Trigger, TriggerResponse
import rospy
import numpy as np
import os
import csv
from math import *
import json


class PolicyEvaluator():
    def __init__(self):
        """
            Each combination robot-human is run serveral times
            All robot and human pomdps have to be defined in the config.json
        """
        # Call the constructor of the parent class

        rospy.Service('~retrieve_policies', Trigger, self.retrieve_policies)
        # rospy.Subscriber("/task_manager/task_status", TaskState, self.evaluate)
        #rospy.Subscriber("/task_manager/task_status", TaskState, schedule)

        self.useEvaluator = False

        json_data= open("../../../../../configs/scenario_config.json").read()
        data = json.loads(json_data)
        #useEvaluator = data["operation_modes.useEvaluator"]
        humans = list(data['evaluation_models']['humans'])
        #self.humans = os.listdir("../../../../../models/human_models/Evaluate/")
        self.humans = list(filter(lambda x: self.isPOMDPX(x), humans))
        policies = list(data['evaluation_models']['policies'])
        #self.policies = os.listdir("../../../../../models/robot_models/Evaluate/")
        self.policies = list(filter(lambda x: self.isPOMDPX(x), policies))
        self.combinations = [(x,y) for x in self.policies for y in self.humans]
        # self.observations = []
        # self.obs = []
        self.scenario_counter = 0
        self.taskNumber = 0
        rospy.set_param('/training_reset', False) # this is to allow manipulating the training tests
        rospy.set_param('/scenario_count', 0) # scenarios for evaluation mode starts from the first human-robot combination. But can be changed manually
        #sampleNumber = 10

    # Each combination robot-human is run serveral times
    # All robot and human pomdps have to be defined in the config.json
    def retrieve_policies(self, req):
        """
    	Callback function:
    		Run every combination of human-robot for several times defined in the parameter 'samples' in the json file.
    		Whenever a new task is this service is called by the task manager
    	@param req a trigger request
    	"""
        # This may not be useful as it is checked already under the TaskManager
        while not rospy.has_param('/evaluator_flag'):
            continue
        self.useEvaluator = rospy.get_param('/evaluator_flag')

        if not self.useEvaluator:
            return

        while not rospy.has_param('/interaction_sample_amount'):
            continue
        self.sampleNumber = rospy.get_param('/interaction_sample_amount')
        while not rospy.has_param('/task_count'):
            continue
        self.taskNumber = rospy.get_param('/task_count')
        while not rospy.has_param('/scenario_count'):
            continue
        self.scenario_counter = rospy.get_param('/scenario_count')

        rospy.loginfo("[Policy Evaluator]: taskNumber = %d and sampleNumber = %d", self.taskNumber, self.sampleNumber)
        # task number starts from 1 (coming from task manager)
        if (((self.taskNumber-1) % self.sampleNumber) == 0 or rospy.get_param('/training_reset')):
            if (self.scenario_counter == len(self.combinations)):
                return TriggerResponse(False, 'All policies are evaluated !')
            rospy.loginfo("[Policy Evaluator]: Changing models")
            pair = self.combinations[self.scenario_counter]
            rospy.set_param("/human_type", pair[1])
            rospy.set_param("/robot_model", pair[0])
            self.scenario_counter = self.scenario_counter + 1
            # in case it is reset from outside
            rospy.set_param('/training_reset', False)
            rospy.set_param('/scenario_count', self.scenario_counter)

        return TriggerResponse(True, 'Policies are provided')

    def isPOMDPX(self, x):
        """
    	Helperfunction:
    		checks if the filename is a pomdpx file, by checking the extension
    	@param x a filename
    	"""
        return x.lower().endswith("pomdpx")

    '''
    def schedule(TaskState):
        """
    	Callback function:
    		Run specific humans in a certain order for evaluation purposes
    	@param TaskState The message published by the /task_manager/task_status
    	"""
    	global humans, policies, obs, i, taskNumber, combinations, useEvaluator
    	humans = [#"distracted_collaborative.POMDPx",
    				# "expert_non_tired_collaborative.POMDPx",
    				 "expert_non_tired_non_collaborative.POMDPx",
    				 "beginner_tired_non_collaborative.POMDPx",
    				 "beginner_tired_collaborative.POMDPx",
    				 "beginner_non_tired_non_collaborative.POMDPx",
    				 #"expert_tired_collaborative.POMDPx",
    				 #"expert_tired_non_collaborative.POMDPx",
    				 #"humanModel_v2.POMDPx",
    				 #"beginner_non_tired_collaborative.POMDPx"
    				 ]
    	if TaskState.task_status in ["START"]:
    		global observations
    		if taskNumber%30 == 0:
    			i=(i+1)%len(humans)
    			rospy.loginfo("[Policy Evaluator]: Change models ")
    			#rospy.loginfo("[Policy Evaluator]:  "+ pair[0] ", " +pair[1])
    			rospy.set_param("/human_type", humans[i])
    		global n
    		taskNumber = taskNumber +1
    '''

if __name__ == "__main__":
    rospy.init_node("policy_evaluator")
    rospy.wait_for_service('/task_manager/new_scenario_request')
    pol_eval = PolicyEvaluator()
    rospy.loginfo("policy evaluator ready!")
    rospy.spin()

	#rospy.set_param("/human_type", "humanModel_expert.POMDPx")
	#rospy.set_param("/robot_model", "proactive_robot_pomdp.pomdpx")
