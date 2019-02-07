# NOTE: Script only works with pyhton2.7
# Dependency: pandas needs to be installed for python 2.7 e.g. by  python2.7 -m pip install pandas


import os
import subprocess
import pandas as pd 
import rospkg
import datetime 
import shutil
import errno

# compile all needed path based on the hrc_ros package path ||  hrc_path: /home/elia/master_thesis/catkin_ws/src/hrc_industry/code/ros_ws/src/hrc_ros
rospack = rospkg.RosPack()
hrc_path = rospack.get_path('hrc_ros')
pomdp_base_path = hrc_path + "/../../../"
pomdp_solver_path = pomdp_base_path + "despot_POMDP_robot/build/examples/pomdpx_models/despot_pomdpx -m " 
pomdp_model_base_path = pomdp_base_path + "models/robot_models/"
results_path = pomdp_base_path + "results/POMDP_IE_tests/"

print("hrc_path: " + hrc_path)
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
		print("********* now executing ************")
		print("POMDP_model : " + current_pomdp_model_str)
		print(" Sequence : " +current_test_sequence_str)
		print("Test : " + current_test_no_str)
		print("row : " + str(row) + "\n\n") 
		# build pomdp path and execute model 
		pomdp_model_path = pomdp_model_base_path + current_pomdp_model_str +"' &"
		pomdp_model_str = "gnome-terminal -e '" + pomdp_solver_path + pomdp_model_path

		time_tag = datetime.datetime.now()
		current_time_str = str(time_tag.date()) + "_" + '%02d'%(time_tag.hour) + ":" + '%02d'%(time_tag.minute)    # str(time_tag.hour) + ":" + str(time_tag.minute)
		result_file_name = "pomdp_evaluator_file_" + current_time_str + ".csv" 
		result_file_path = results_path + "pomdp_evaluator_file_" + str(time_tag.date()) + "_" + str(time_tag.hour) + ":" + str(time_tag.minute) + ".csv" 

		os.system(pomdp_model_str)
		os.system("sleep 5")
		# execute test script
		test_sequ_str = hrc_path + "/../../../pomdp_evaluation/stimuli/" + current_test_sequence_str
		execfile(test_sequ_str)

		# ***** rename and copy the result files to a folder ******* 
		new_folder_path = results_path + "Test_" + current_test_no_str
		new_file_name = current_pomdp_model_str + "_" + current_test_sequence_str + "_" + current_test_no_str + "_" + current_time_str + ".csv"

		if not os.path.exists(new_folder_path):
			try:
				os.makedirs(new_folder_path, 0o700)
			except OSError as e:
				if e.errno != errno.EEXIST:
					raise 
		shutil.copy(result_file_path,new_folder_path)
		os.rename(new_folder_path + "/" + result_file_name, new_folder_path + "/" + new_file_name )
		print(new_file_name)
		# sleep to ensure that next test is labeled with +1 min 
		os.system('killall despot_pomdpx')
		os.system("sleep 65") 

os.system('killall despot_pomdpx')


#proc = subprocess.Popen(['gnome-terminal','-e',' rosrun hrc_ros robot_agent_pomdp_evaluation & '], shell=True)
#print proc.pid

#os.system("sleep 60")

#proc.terminate
#os.system('rosnode kill /robot_agent')
#os.system("gnome-terminal -e 'rosrun hrc_ros robot_agent_pomdp_evaluation' & ")
#pid_to_kill = os.system(" echo $!")
#os.system(" echo Blubblub") 

#print pid_to_kill 

#kill_cmd = "kill " + str(pid_to_kill)  
#os.system(kill_cmd)



#execfile("/home/elia/master_thesis/catkin_ws/src/hrc_industry/code/ros_ws/src/hrc_ros/src/test_agents_IE/pomdp_client_obsonly.py")
#os.system('killall despot_pomdpx')
#os.system("sleep 65") 


#os.system("gnome-terminal -e '/home/elia/master_thesis/catkin_ws/src/hrc_industry/code/despot_POMDP_robot/build/examples/pomdpx_models/despot_pomdpx -m /home/elia/master_thesis/catkin_ws/src/hrc_industry/code/models/robot_models/proactive_IE_minimalChanges.pomdpx' &")
#os.system("sleep 5")

#execfile("/home/elia/master_thesis/catkin_ws/src/hrc_industry/code/ros_ws/src/hrc_ros/src/test_agents_IE/pomdp_client_obsonly.py")
#os.system('killall despot_pomdpx')
#os.system('rosnode kill /robot_agent')
