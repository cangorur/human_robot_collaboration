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
files_not_copied = 0


# compile all needed path based on the hrc_ros package path ||  hrc_path: /home/elia/master_thesis/catkin_ws/src/hrc_industry/code/ros_ws/src/hrc_ros
# TODO CHANGE PATH HERE
rospack = rospkg.RosPack()
hrc_path = rospack.get_path('hrc_ros')
pomdp_base_path = hrc_path + "/../../../"
pomdp_solver_path = pomdp_base_path + "despot_POMDP_robot/build/examples/pomdpx_models/despot_pomdpx -m " 
pomdp_model_base_path = pomdp_base_path + "models/robot_models/"
results_path = pomdp_base_path + "results/POMDP_IE_tests/"
pomdp_in_own_terminal = False # set true if pomdp should be launched in own terminal (this will always be in the foreground )




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
		if (pomdp_in_own_terminal == True):
			pomdp_model_path = pomdp_model_base_path + current_pomdp_model_str +"' &" 
			pomdp_execute_command = "gnome-terminal -e '" + pomdp_solver_path + pomdp_model_path

		elif (pomdp_in_own_terminal == False):
			pomdp_model_path = pomdp_model_base_path + current_pomdp_model_str 
			pomdp_execute_command = pomdp_solver_path + pomdp_model_path + " &"
		

		print("executing : " + str(pomdp_execute_command) )
		time_tag = datetime.datetime.now()
		os.system(pomdp_execute_command)
		current_time_str = str(time_tag.date()) + "_" + '%02d'%(time_tag.hour) + ":" + '%02d'%(time_tag.minute) + "_" + '%02d'%(time_tag.second)   # str(time_tag.hour) + ":" + str(time_tag.minute)
		result_file_name = "pomdp_evaluator_file_" + current_time_str + ".csv" 
		result_file_path = results_path + "pomdp_evaluator_file_" + current_time_str + ".csv" 
		
		

		os.system("sleep 2")
		# execute test script
		# TODO CHANGE PATH HERE
		test_sequ_str = hrc_path + "/../../../pomdp_evaluation/stimuli/" + current_test_sequence_str
		execfile(test_sequ_str)

		# ***** rename and copy the result files to a folder ******* 
		# TODO CHANGE PATH HERE
		new_folder_path = results_path + "Test_" + current_test_no_str
		new_file_name = current_pomdp_model_str + "__" + current_test_sequence_str + "__" + "Test" + current_test_no_str + "__" + current_time_str + ".csv"

		print ( " \n The script was started at" + str(current_time_str))

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
			try: # increment second by 1 and try to copy again	
			# TODO paths can be changed but don't have to be 
				print("increment by 1 sec")
				new_tag = time_tag + datetime.timedelta(0,1) # add 1 second
				new_time_str = str(new_tag.date()) + "_" + '%02d'%(new_tag.hour) + ":" + '%02d'%(new_tag.minute) + "_" + '%02d'%(new_tag.second) 
				new_res_file_name = "pomdp_evaluator_file_" + new_time_str + ".csv"
				print("new file I try to copy  " + str(new_res_file_name))
				new_file_name = current_pomdp_model_str + "__" + current_test_sequence_str + "__" + "Test"+ current_test_no_str + "__" + new_time_str + ".csv"
				result_file_path = results_path + "pomdp_evaluator_file_" + new_time_str + ".csv" 
				shutil.copy(result_file_path,new_folder_path)
				os.rename(new_folder_path + "/" + new_res_file_name, new_folder_path + "/" + new_file_name )
				print("Incremented result_file is : " + new_res_file_name)
				print("result_file_path : " + result_file_path) 
			except: 
				try: # increment second by 1 and try to copy again	
					print("increment by 2 sec")
					new_tag = time_tag + datetime.timedelta(0,2) # add 2 seconds
					new_time_str = str(new_tag.date()) + "_" + '%02d'%(new_tag.hour) + ":" + '%02d'%(new_tag.minute) + "_" + '%02d'%(new_tag.second)  
					new_res_file_name = "pomdp_evaluator_file_" + new_time_str + ".csv"
					print("new file I try to copy  " + str(new_res_file_name))
					new_file_name = current_pomdp_model_str + "__" + current_test_sequence_str + "__" + "Test"+ current_test_no_str + "__" + new_time_str + ".csv"
					result_file_path = results_path + "pomdp_evaluator_file_" + new_time_str + ".csv" 
					shutil.copy(result_file_path,new_folder_path)
					os.rename(new_folder_path + "/" + new_res_file_name, new_folder_path + "/" + new_file_name )
					print("Incremented result_file is : " + new_res_file_name)
					print("result_file_path : " + result_file_path) 
				except:
					try: # increment second by 1 and try to copy again	
						print("increment by 3 sec")
						new_tag = time_tag + datetime.timedelta(0,3) # add 3 seconds
						new_time_str = str(new_tag.date()) + "_" + '%02d'%(new_tag.hour) + ":" + '%02d'%(new_tag.minute) + "_" + '%02d'%(new_tag.second)  
						new_res_file_name = "pomdp_evaluator_file_" + new_time_str + ".csv"
						print("new file I try to copy  " + str(new_res_file_name))
						new_file_name = current_pomdp_model_str + "__" + current_test_sequence_str + "__" + "Test"+ current_test_no_str + "__" + new_time_str + ".csv"
						result_file_path = results_path + "pomdp_evaluator_file_" + new_time_str + ".csv" 
						shutil.copy(result_file_path,new_folder_path)
						os.rename(new_folder_path + "/" + new_res_file_name, new_folder_path + "/" + new_file_name )
						print("Incremented result_file is : " + new_res_file_name)
						print("result_file_path : " + result_file_path) 
					except: 
						try: # increment second by 1 and try to copy again	
							print("increment by 4 sec")
							new_tag = time_tag + datetime.timedelta(0,4) # add 3 seconds
							new_time_str = str(new_tag.date()) + "_" + '%02d'%(new_tag.hour) + ":" + '%02d'%(new_tag.minute) + "_" + '%02d'%(new_tag.second)  
							new_res_file_name = "pomdp_evaluator_file_" + new_time_str + ".csv"
							print("new file I try to copy  " + str(new_res_file_name))
							new_file_name = current_pomdp_model_str + "__" + current_test_sequence_str + "__" + "Test"+ current_test_no_str + "__" + new_time_str + ".csv"
							result_file_path = results_path + "pomdp_evaluator_file_" + new_time_str + ".csv" 
							shutil.copy(result_file_path,new_folder_path)
							os.rename(new_folder_path + "/" + new_res_file_name, new_folder_path + "/" + new_file_name )
							print("Incremented result_file is : " + new_res_file_name)
							print("result_file_path : " + result_file_path) 
						except: 
							try: # increment second by 1 and try to copy again
								print("increment by 5 sec")	
								new_tag = time_tag + datetime.timedelta(0,5) # add 3 seconds
								new_time_str = str(new_tag.date()) + "_" + '%02d'%(new_tag.hour) + ":" + '%02d'%(new_tag.minute) + "_" + '%02d'%(new_tag.second)  
								new_res_file_name = "pomdp_evaluator_file_" + new_time_str + ".csv"
								print("new file I try to copy  " + str(new_res_file_name))
								new_file_name = current_pomdp_model_str + "__" + current_test_sequence_str + "__" + "Test"+ current_test_no_str + "__" + new_time_str + ".csv"
								result_file_path = results_path + "pomdp_evaluator_file_" + new_time_str + ".csv" 
								shutil.copy(result_file_path,new_folder_path)
								os.rename(new_folder_path + "/" + new_res_file_name, new_folder_path + "/" + new_file_name )
								print("Incremented result_file is : " + new_res_file_name)
								print("result_file_path : " + result_file_path) 
							except: 
								print(" ERROR in 6th trial except = file could not be copied ")
								files_not_copied += 1
	
		
		print("new File to copy: " + new_file_name + "\n")
		# sleep to ensure that next test is labeled with +1 min 
		os.system('killall despot_pomdpx')
		kill_test_str = "killall " + current_test_sequence_str
		os.system(kill_test_str)
		print(" ... waiting for 2 seconds .. ")
		os.system("sleep 2") 

os.system('killall despot_pomdpx')

finish_time = datetime.datetime.now()
time_taken = finish_time - start_time
print("\n\n I'm done with this :-) All tests executed\n\nTook me: hh:mm:ss:microseconds " + str(time_taken))
print("\n # of files I could not copy : " + str(files_not_copied))


