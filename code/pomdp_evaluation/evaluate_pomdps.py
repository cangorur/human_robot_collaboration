# Copyright 2019 Elia Kargruber (elitscheri) 
#
# This project is licensed under the terms of the MIT license.
# 
# NOTE: Script only works with pyhton2.7
# Dependency: pandas needs to be installed for python 2.7 e.g. by  python2.7 -m pip install pandas
# Paths have to be adapted to your local project -> change everywhere whre you find a # TODO CHANGE PATH HERE tag 


import os
import subprocess
import pandas as pd 
import rospkg
import datetime 
import shutil
import errno

# compile all needed path based on the hrc_ros package path ||  hrc_path: /home/elia/master_thesis/catkin_ws/src/hrc_industry/code/ros_ws/src/hrc_ros
# TODO CHANGE PATH HERE
rospack = rospkg.RosPack()
hrc_path = rospack.get_path('hrc_ros')
pomdp_base_path = hrc_path + "/../../../"
pomdp_solver_path = pomdp_base_path + "despot_POMDP_robot/build/examples/pomdpx_models/despot_pomdpx -m " 
pomdp_model_base_path = pomdp_base_path + "models/robot_models/"
results_path = pomdp_base_path + "results/POMDP_IE_tests/"

# get the beginning time to measure how long it takes
start_time = datetime.datetime.now()


# read in test configuration 
test_config = pd.read_csv("POMDP_evaluation_config.csv")
#os.system("rosrun hrc_ros robot_agent_pomdp_evaluation &")   # robot_agent cannot be killed savely -> do not use for now 



# execute POMDP model with associated test_case row by row 
l = test_config.index.values
for row in (test_config.index):
#for row in range(1,len(l)): 

	print(row)	

	if (row >= 0): 
		os.system('killall despot_pomdpx')

		current_pomdp_model_str = str(test_config['POMDP_MODEL'][row])
		current_test_sequence_str = str(test_config['TEST_SEQUENCE'][row])
		current_test_no_str       = str(test_config['TEST_NO'][row])
		print("\n********* now executing ************")
		print("POMDP_model : " + current_pomdp_model_str)
		print(" Sequence : " +current_test_sequence_str)
		print("Test : " + current_test_no_str)
		print("row : " + str(row) ) 

		# build pomdp path and execute model
		# TODO CHANGE PATH HERE 
		pomdp_model_path = pomdp_model_base_path + current_pomdp_model_str +"' &"
		pomdp_model_str = "gnome-terminal -e '" + pomdp_solver_path + pomdp_model_path

		time_tag = datetime.datetime.now()
		os.system(pomdp_model_str)
		current_time_str = str(time_tag.date()) + "_" + '%02d'%(time_tag.hour) + ":" + '%02d'%(time_tag.minute)    # str(time_tag.hour) + ":" + str(time_tag.minute)
		result_file_name = "pomdp_evaluator_file_" + current_time_str + ".csv" 
		result_file_path = results_path + "pomdp_evaluator_file_" + current_time_str + ".csv" 
		

		os.system("sleep 5")
		# execute test script
		# TODO CHANGE PATH HERE
		test_sequ_str = hrc_path + "/../../../pomdp_evaluation/stimuli/" + current_test_sequence_str
		execfile(test_sequ_str)

		# ***** rename and copy the result files to a folder ******* 
		# TODO CHANGE PATH HERE
		new_folder_path = results_path + "Test_" + current_test_no_str
		new_file_name = current_pomdp_model_str + "__" + current_test_sequence_str + "__" + "Test" + current_test_no_str + "__" + current_time_str + ".csv"


		print(" ____________ Results __________")
		if not os.path.exists(new_folder_path):
			try:
				os.makedirs(new_folder_path, 0o700)
			except OSError as e:
				print("OSError while making directories")
				print(e)
			except:
				print("Unexpected error while making directories:")
			else: 
				print (" In else branch while making directories" )

		try:
			print("evaluator_file : " + result_file_name )
			shutil.copy(result_file_path,new_folder_path)
			os.rename(new_folder_path + "/" + result_file_name, new_folder_path + "/" + new_file_name )
		except:
			print("Unexpected error while copying files:")
			print(" Had to increment minute by 1 - will try again ") 
			try: # increment minute by 1 and try to copy againn 	
			# TODO paths can be changed but don't have to be 
				new_tag = time_tag + datetime.timedelta(0,60) # add 60 seconds 
				new_time_str = str(new_tag.date()) + "_" + '%02d'%(new_tag.hour) + ":" + '%02d'%(new_tag.minute)
				new_res_file_name = "pomdp_evaluator_file_" + new_time_str + ".csv"
				new_file_name = current_pomdp_model_str + "__" + current_test_sequence_str + "__" + "Test"+ current_test_no_str + "__" + new_time_str + ".csv"
				result_file_path = results_path + "pomdp_evaluator_file_" + new_time_str + ".csv" 
				shutil.copy(result_file_path,new_folder_path)
				os.rename(new_folder_path + "/" + new_res_file_name, new_folder_path + "/" + new_file_name )
				print("Incremented result_file is : " + new_res_file_name)
				print("result_file_path : " + result_file_path) 
			except: 
				print("in second trial except = file could not be copied")
	
		
		print("new File to copy: " + new_file_name + "\n")
		# sleep to ensure that next test is labeled with +1 min 
		os.system('killall despot_pomdpx')
		kill_test_str = "killall " + current_test_sequence_str
		os.system(kill_test_str)
		print(" ... waiting for 1min .. ")
		os.system("sleep 65") 

os.system('killall despot_pomdpx')

finish_time = datetime.datetime.now()
time_taken = finish_time - start_time
print("\n\n I'm done with this :-) All tests executed\n\nTook me: ")
print(time_taken)
print("\n\n I'm done with this :-) All tests executed\n\nTook me: hh:mm:ss:microseconds " + str(time_taken))


