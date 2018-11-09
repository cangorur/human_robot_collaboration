## Paths that are should be changed first;

"config/scenario_config.json" file 	:	training_set folder path should be changed; "policySelectorsTrainingSet" and "trainingSetName"
../hrc_ros/src/BPR_EI/BPR_training_set_creation.py: 	config file path should be changed
../hrc_ros/src/BPR_EI/BPR_policy_selector.py  	  :	config file path should be changed

##



In ../hrc_ros/src/BPR_EI folder, there are 2 nodes called BPR_trainer and BPR_policy_selector

# How to run Policy Selector
'''
rosrun hrc_ros <BPR_policy_selector.py>
'''
This will initiate the policy_selector node and this hode has a subscriber to task_manager/task_status topic.

To publish /task_manager/task_status topic
Either use:
'''
rosbag play tests_{EXPERIMENT_DATE_TIME}.bag
'''
Or project could be run by MORSE. This will also publish the same topic.

Then to select a policy;

'''
rosservice call /select_policy "True"
'''

Service will use "observation_vector and total_discounted_reward" of the last completed task and return a policy number then updates the belief over human types.


## How to create a Training Set to be used in Policy Selector

'''
rosrun hrc_ros <BPR_training_set_creation.py>
'''
This will initiate the BPR_Trainer node and this hode has a subscriber to task_manager/task_status topic.

'''
rosservice call /train_algorithm
'''

Service will start the training set creation process and ".mat" file will be created at the end of the experiment

To publish /task_manager/task_status topic
Either use:
'''
rosbag play tests_{EXPERIMENT_DATE_TIME}.bag
'''
Or project could be run by MORSE. This will also publish the same topic.


