import csv
import sys
import numpy as np

# added for analysis for IE
import os
import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator, MultipleLocator, FuncFormatter
from scipy import stats
#from researchpy import rp

def save_sensor_data(dir, raw):
    for col_index in range(len(raw[0])):
        if raw[0][col_index] == "who_reports":
            col_reporter = col_index
        elif raw[0][col_index] == "task_status":
            col_taskSt = col_index

    with open(dir + '/sensor_update.csv', 'a') as file:
        wr = csv.writer(file, dialect='excel')
        wr.writerow(raw[0])

        for row_index in range(len(raw)):
            if raw[row_index][col_reporter] == '"MANAGER"':
                wr.writerow(raw[row_index])
            elif raw[row_index][col_reporter] == '"MANAGER-TASK-DONE"':
                wr.writerow(raw[row_index])
            elif raw[row_index][col_reporter] == '"SENSOR"':
                wr.writerow(raw[row_index])
    file.close()
    print('INFO: sensor information is saved separately under the folder provided')

def human_observables(dir, raw):

    for col_index in range(len(raw[0])):
        if raw[0][col_index] == "task_id":
            col_taskID = col_index
        elif raw[0][col_index] == "step_count":
            col_step = col_index
        elif raw[0][col_index] == "update_received_time":
            col_update_time = col_index
        elif raw[0][col_index] == "who_reports":
            col_whoRep = col_index
        elif raw[0][col_index] == "human_model":
            col_humanModel = col_index
        elif raw[0][col_index] == "human_observables":
            col_humanObs = col_index

    with open(dir + '/human_observables.csv', 'a') as file:
        wr = csv.writer(file, dialect='excel')
        first_row = ["task_id", "step_count", "who_reports", "update_received_time", "update_secs", "update_nsecs", "human_model", "human_observables",
                    "Human Detected", "Looked Around", "Succesfull Grasp", "Failed Grasp", "Warned Robot", "Stood Idle"]

        wr.writerow(first_row)
        new_row = [None] * 14  # initiate a row

        for row_index in range(len(raw)):
            if row_index == 0:  # skip the first row which has titles only
                continue
            if raw[row_index][col_whoRep] == '"MANAGER"':
                human_model = raw[row_index][col_humanModel]
            if raw[row_index][col_whoRep] == '"OBSERVATION"':
                new_row[0] = raw[row_index][col_taskID]
                new_row[1] = raw[row_index][col_step]
                new_row[2] = raw[row_index][col_whoRep]
                new_row[3] = raw[row_index][col_update_time]
                new_row[4] = raw[row_index][col_update_time + 1]
                new_row[5] = raw[row_index][col_update_time + 2]
                new_row[6] = human_model
                new_row[7] = raw[row_index][col_humanObs]
                # separating a string (was a boolean vector) to each elements for the human observables
                obs_array_str = raw[row_index][col_humanObs]
                det, la, sucGra, failGra, warn, idle = obs_array_str.split()
                det, det0 = det.split(',')
                la, la0 = la.split(',')
                sucGra, sucGra0 = sucGra.split(',')
                failGra, failGra0 = failGra.split(',')
                warn, warn0 = warn.split(',')

                if det[1:] == 'True': det = 1
                else: det = 0
                if la == 'True': la = 1
                else: la = 0
                if sucGra == 'True': sucGra = 1
                else: sucGra = 0
                if failGra == 'True': failGra = 1
                else: failGra = 0
                if warn == 'True': warn = 1
                else: warn = 0
                if idle[:-1] == 'True': idle = 1
                else: idle = 0

                new_row[8] = det
                new_row[9] = la
                new_row[10] = sucGra
                new_row[11] = failGra
                new_row[12] = warn
                new_row[13] = idle

                wr.writerow(new_row)

    file.close()
    print('INFO: human observables are saved separately under the folder provided')

# get all files in a directory 
def getFileList(dirName):
    # create a list of file and sub directories 
    # names in the given directory 
    listOfFile = os.listdir(dirName)
    allFiles = list()
    # Iterate over all the entries
    for entry in listOfFile:
        # Create full path
        fullPath = os.path.join(dirName, entry)
        # If entry is a directory then get the list of files in this directory 
        if os.path.isdir(fullPath):
            allFiles = allFiles + getFileList(fullPath)
        else:
            allFiles.append(fullPath)
                
    return allFiles

# I have to make sure manually that each task has a sensor reading reporting the success
def task_status(dir):
    with open(dir + '/sensor_update.csv', 'rb') as file:
        reader = csv.reader(file, delimiter=',')
        sensor_updates = list(reader)

        for col_index in range(len(sensor_updates[0])):
            if sensor_updates[0][col_index] == "task_id":
                col_taskID = col_index
            elif sensor_updates[0][col_index] == "who_reports":
                col_whoRep = col_index
            elif sensor_updates[0][col_index] == "update_received_time":
                col_update_time = col_index
            elif sensor_updates[0][col_index] == "subtask_status":
                col_subtaskSt = col_index
            elif sensor_updates[0][col_index] == "who_succeeded_subtask":
                col_whoSuc = col_index
            # Additional columns used for the interaction experiment case 
            elif sensor_updates[0][col_index] == "subtask_duration":
                col_subtDuration = col_index 
            elif sensor_updates[0][col_index] == "task_duration":
                col_taskDuration = col_index 


        with open(dir + '/task_status.csv', 'a') as new_file:
            wr = csv.writer(new_file, dialect='excel')
            # first row:
            first_row = ["task_id", "init_secs", "init_nsecs", "fin_secs", "fin_nsecs", "time_took", "moving_avg_time",
            "task_status", "moving_avg_status", "who_succeeded", "blub"]
            wr.writerow(first_row)
            new_row = [None] * 12  # initiate a row
            manager_flag = False
            finish_report_flag = False
            cumulative_timeTook = 0.0
            cumulative_success = 0.0
            for row_index in range(len(sensor_updates)):

                if row_index == 0:  # skip the first row which has titles only
                    continue
                if sensor_updates[row_index][col_whoRep] == '"MANAGER"':
                    new_row[0] = sensor_updates[row_index][col_taskID]
                    new_row[1] = sensor_updates[row_index][col_update_time + 1]
                    new_row[2] = sensor_updates[row_index][col_update_time + 2]
                    manager_flag = True
                if sensor_updates[row_index][col_whoRep] == '"SENSOR"':  #and \ new_row[0] == sensor_updates[row_index][ col_taskID]:  # if the same task with manager report
                    #new_row[3] = sensor_updates[row_index][col_update_time + 1]
                    #new_row[4] = sensor_updates[row_index][col_update_time + 2]
                    #init_time = new_row[1] + '.' + new_row[2]
                    #fin_time = new_row[3] + '.' + new_row[4]
                    #new_row[5] = 0 #float(fin_time) - float(init_time)
                    #cumulative_timeTook = cumulative_timeTook + float(fin_time) - float(init_time)
                    #new_row[6] = float(cumulative_timeTook / int(sensor_updates[row_index][col_taskID])) # moving avg time
                    new_row[7] = 4
                    if sensor_updates[row_index][col_subtaskSt] == '"success"':
                        new_row[7] = 1
                    if sensor_updates[row_index][col_subtaskSt] == '"fail"':
                        new_row[7] = 0
                    if sensor_updates[row_index][col_subtaskSt] == '"ROBOT SUCCEEDED"':
                        new_row[7] = 1
                        #new_row[10] = 1
                    print(cumulative_success,new_row[7])
                    cumulative_success = cumulative_success + new_row[7]
                    new_row[8] = float(cumulative_success / int(sensor_updates[row_index][col_taskID]))
                    new_row[9] = sensor_updates[row_index][col_whoSuc]
                    finish_report_flag = True

                if manager_flag and finish_report_flag:
                    wr.writerow(new_row)  # finished a row for a task
                    new_row = [None] * 10  # initiate a row
                    manager_flag = False
                    finish_report_flag = False

        new_file.close()
    file.close()
    print('INFO: task status information is saved separately under the folder provided')


def filter_csvs(dir):
    file_list = getFileList(dir)
    for result_file in file_list: 
        types_dict = {'rosbagTimestamp':str, 'task_id':int, 'subtask_id':int,	'step_count':int, 'who_reports':str, 'update_received_time':str,	'secs1':int,	'nsecs1':int, 'human_model':str,	'action_taken_time':str , 'secs':int , 'nsecs':int, 'taken_action':str, 'belief_state':str, 'real_state':str, 'isEstimationCorrect':str,	'warnings_count_subtask':int, 'real_obs_received':str, 'real_obs_received_array':str, 'obs_with_noise':str,'human_observables':str, 'mapped_observation_pomdp':int,	'mapped_observation_raw':int, 'robot_model':str, 'immediate_reward':str, 'total_disc_reward':str,'robot_belief':str,	'tray_update_received_time':str, 'secs':int,	'nsecs':int, 'subtask_status':str,	'who_succeeded_subtask':str,	'subtask_duration':float,	'failed_subtasks':int, 'successful_subtasks':int, 'percentage_successful_subtasks':float,	'task_status':str,	'task_duration':float,	'who_succeeded_task':str,	'warnings_count_task':int,	'successful_tasks_cnt':int,	'failed_tasks_cnt':int,	'percentage_successful_tasks':float}

        file_frame = pd.read_csv(result_file, dtype = types_dict)
        
        # #### filter csv files 
        file_frame.drop(['human_model','update_received_time','secs.1',	'nsecs.1', 'action_taken_time', 'secs', 'nsecs','real_state','isEstimationCorrect','obs_with_noise','robot_belief','tray_update_received_time','secs','nsecs','who_succeeded_task'],axis=1,inplace= True)
        print(result_file)
        
        filtered_file = ((result_file.strip(dir)).replace('.csv','')) + str("_filtered.csv")
        file_frame.to_csv(dir + str("/../task_success_files_filtered/") + filtered_file)


# ##########################################################################################################################
# This function plots the characterstics of the challenging tasks py parsing a number of csv files and averaging over them 
# Use like this python IntExperiment_update_sensor_data.py ./Participants/Analysis/task_success_files plot_challenging 1 1 
# The trailing booleans are    debug  &  show_plots    control variables 

def plot_challenging(dir,print_debug_sys,show_plots_sys):

    print(print_debug_sys)
    print(show_plots_sys)
    if int(print_debug_sys) == 1: 
        print_debug = True # defines if debug info is printed (e.g. mean arrays ... )
    else: 
        print_debug = False 
    if int(show_plots_sys) == 1:
        show_plots = True  # defines if plots are shown, if false they are only saved 
    else: 
        show_plots = False 

    file_list = getFileList(dir)
    
    if(print_debug == True):
        print("\n")
        print(file_list)
        print("\n")
    
    # variables for analysis   ttype = tasktype 
    ttype1_rewards = np.array([], dtype=np.float64)
    ttype23_rewards = np.array([], dtype=np.float64)
    ttype24_rewards = np.array([], dtype=np.float64)
    ttype25_rewards = np.array([], dtype=np.float64)
    ttype26_rewards = np.array([], dtype=np.float64)
    ttype47_rewards = np.array([], dtype=np.float64)
    ttype49_rewards = np.array([], dtype=np.float64)
    ttype5_rewards = np.array([], dtype=np.float64)
    ttype2_rewards = np.array([], dtype=np.float64)

    ttype1_task_duration = np.array([], dtype=np.float64)
    ttype2_task_duration = np.array([], dtype=np.float64)
    ttype23_task_duration = np.array([], dtype=np.float64)
    ttype4_task_duration = np.array([], dtype=np.float64)
    ttype47_task_duration = np.array([], dtype=np.float64)
    ttype5_task_duration = np.array([], dtype=np.float64)

    ttype1_robot_takeover = np.array([], dtype=np.float64)
    ttype2_robot_takeover = np.array([], dtype=np.float64)
    ttype4_robot_takeover = np.array([], dtype=np.float64)

    ttype1_percentage_correct_subt = np.array([], dtype=np.float64)
    ttype1_robot_tookover = np.array([], dtype=np.float64)
    ttype1_task_duration = np.array([], dtype=np.float64)
    

    for result_file in file_list: 
        types_dict = {'rosbagTimestamp':str, 'task_id':int, 'subtask_id':int,	'step_count':int, 'who_reports':str, 'update_received_time':str,	'secs1':int,	'nsecs1':int, 'human_model':str,	'action_taken_time':str , 'secs':int , 'nsecs':int, 'taken_action':str, 'belief_state':str, 'real_state':str, 'isEstimationCorrect':str,	'warnings_count_subtask':int, 'real_obs_received':str, 'real_obs_received_array':str, 'obs_with_noise':str,'human_observables':str, 'mapped_observation_pomdp':int,	'mapped_observation_raw':int, 'robot_model':str, 'immediate_reward':str, 'total_disc_reward':str,'robot_belief':str,	'tray_update_received_time':str, 'secs':int,	'nsecs':int, 'subtask_status':str,	'who_succeeded_subtask':str,	'subtask_duration':float,	'failed_subtasks':int, 'successful_subtasks':int, 'percentage_successful_subtasks':float,	'task_status':str,	'task_duration':float,	'who_succeeded_task':str,	'warnings_count_task':int,	'successful_tasks_cnt':int,	'failed_tasks_cnt':int,	'percentage_successful_tasks':float}
        #file_frame = pd.read_csv(result_file,dtype = types_dict, names=['rosbagTimestamp', 'task_id', 'subtask_id',	'step_count', 'who_reports', 'update_received_time',	'secs1',	'nsecs1', 'human_model',	'action_taken_time', 'secs', 'nsecs', 'taken_action', 'belief_state', 'real_state', 'isEstimationCorrect',	'warnings_count_subtask', 'real_obs_received', 'real_obs_received_array', 'obs_with_noise','human_observables', 'mapped_observation_pomdp',	'mapped_observation_raw', 'robot_model', 'immediate_reward', 'total_disc_reward','robot_belief',	'tray_update_received_time', 'secs3',	'nsecs3', 'subtask_status',	'who_succeeded_subtask',	'subtask_duration',	'failed_subtasks', 'successful_subtasks', 'percentage_successful_subtasks',	'task_status',	'task_duration',	'who_succeeded_task',	'warnings_count_task',	'successful_tasks_cnt',	'failed_tasks_cnt',	'percentage_successful_tasks'])

        file_frame = pd.read_csv(result_file, dtype = types_dict)
        
        # #### filter csv files 
        file_frame.drop(['human_model','update_received_time','secs.1',	'nsecs.1', 'action_taken_time', 'secs', 'nsecs','real_state','isEstimationCorrect','obs_with_noise','robot_belief','tray_update_received_time','secs','nsecs','who_succeeded_task'],axis=1,inplace= True)
        print(result_file)
        
        #rp.summary_cont(file_frame['who_succeeded_subtask'])
        # files can now be filtered and saved separately by calling this script with the argument filter_csvs 
        #filtered_file = (result_file.strip(dir)).replace('.csv','') + str("_filtered.csv")
        #file_frame.to_csv(dir + str("/../task_success_files_filtered/") + filtered_file)


        if(print_debug == True):
            print(result_file)
        
        turn1_done = 0 
        turn2_done = 0
        turn3_done = 0
        turn6_done = 0
        turn7_done = 0

        for row in (file_frame.index):
            
            task_number = 1
            if ( ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == task_number ) ):
                turn1_done = row
            # turn 2 | task type 1 -> means analysis + SEM for each participant  | additional " in the string need to be trimmed  
            task_number = 2
            if ( ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == task_number ) ): 
                ttype1_rewards = np.append( ttype1_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )
                ttype1_task_duration = np.append( ttype1_task_duration, float((file_frame['task_duration'][row])))
                turn2_done = row

            # turn 3 | task type 2 -> means analysis + SEM for each participant 
            task_number = 3
            if ( ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == task_number ) ): 
                ttype23_rewards = np.append( ttype23_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )
                ttype2_task_duration = np.append( ttype2_task_duration, float((file_frame['task_duration'][row])))
                ttype2_rewards = np.append( ttype2_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )
                ttype23_task_duration = np.append( ttype23_task_duration, float((file_frame['task_duration'][row])))
                turn3_done = row 

            # turn 4 | task type 2 -> means analysis + SEM for each participant 
            task_number = 4
            if ( ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == task_number ) ): 
                ttype24_rewards = np.append( ttype24_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )
                ttype2_task_duration = np.append( ttype2_task_duration, float((file_frame['task_duration'][row])))
                ttype2_rewards = np.append( ttype2_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )

            # turn 5 | task type 2 -> means analysis + SEM for each participant 
            task_number = 5
            if ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == task_number ): 
                ttype25_rewards = np.append( ttype25_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )
                ttype2_task_duration = np.append( ttype2_task_duration, float((file_frame['task_duration'][row])))
                ttype2_rewards = np.append( ttype2_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )

            # turn 6 | task type 2 -> means analysis + SEM for each participant 
            task_number = 6
            if ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == task_number ): 
                ttype26_rewards = np.append( ttype26_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )
                ttype2_task_duration = np.append( ttype2_task_duration, float((file_frame['task_duration'][row])))
                ttype2_rewards = np.append( ttype2_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )
                turn6_done
            # turn 7 | task type 4 -> means analysis + SEM for each participant 
            task_number = 7
            if ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == task_number ): 
                ttype47_rewards = np.append( ttype47_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )
                ttype4_task_duration = np.append( ttype4_task_duration, float((file_frame['task_duration'][row])))
                ttype47_task_duration = np.append( ttype47_task_duration, float((file_frame['task_duration'][row])))
                turn7_done = row
            
            # turn 8 | task type 5 -> means analysis + SEM for each participant 
            task_number = 8
            if ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == task_number ): 
                ttype5_rewards = np.append( ttype5_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )
                ttype5_task_duration = np.append( ttype5_task_duration, float((file_frame['task_duration'][row])))

            # turn 9 | task type 4 -> means analysis + SEM for each participant 
            task_number = 9
            if ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == task_number ): 
                ttype49_rewards = np.append( ttype49_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )
                ttype4_task_duration = np.append( ttype4_task_duration, float((file_frame['task_duration'][row])))
        
        
        
        # calculate robot take over -  task type 1
        robot_succeeded = 0
        human_succeeded = 0
        for idx in range(turn1_done,turn2_done):
            if file_frame['who_succeeded_subtask'][idx] == '"ROBOT"':
                robot_succeeded += 1
            if file_frame['who_succeeded_subtask'][idx] == '"HUMAN"': 
                human_succeeded += 1
        ttype1_robot_takeover  = np.append(ttype1_robot_takeover, robot_succeeded) 
        mean_robot_takeover_ttype1 = np.mean(ttype1_robot_takeover)
        sem_robot_takeover_ttype1 = stats.sem(ttype1_robot_takeover)

        # calculate robot take over -  task type 2
        robot_succeeded = 0
        human_succeeded = 0
        for idx in range(turn2_done,turn3_done):
            if file_frame['who_succeeded_subtask'][idx] == '"ROBOT"':
                robot_succeeded += 1
            if file_frame['who_succeeded_subtask'][idx] == '"HUMAN"': 
                human_succeeded += 1
        ttype2_robot_takeover  = np.append(ttype2_robot_takeover, robot_succeeded)
        mean_robot_takeover_ttype2 = np.mean(ttype2_robot_takeover) 
        sem_robot_takeover_ttype2 = stats.sem(ttype2_robot_takeover)

        # calculate robot take over -  task type 4
        robot_succeeded = 0
        human_succeeded = 0
        for idx in range(turn6_done,turn7_done):
            if file_frame['who_succeeded_subtask'][idx] == '"ROBOT"':
                robot_succeeded += 1
            if file_frame['who_succeeded_subtask'][idx] == '"HUMAN"': 
                human_succeeded += 1
        ttype4_robot_takeover  = np.append(ttype4_robot_takeover, robot_succeeded) 
        mean_robot_takeover_ttype4 = np.mean(ttype4_robot_takeover)
        sem_robot_takeover_ttype4 = stats.sem(ttype4_robot_takeover)

        



    # ANALYZE REWARDS:  MEANS AND SEM     
    ttype1_rewards_sem = stats.sem(ttype1_rewards)
    mean_rewards_ttype1 = np.mean(ttype1_rewards)
    if(print_debug == True):
        print(ttype1_rewards)
        print("\n")
        print("ttype1")
        print(ttype1_rewards)
        print("mean: " + str(mean_rewards_ttype1))
        print("SEM: " + str(ttype1_rewards_sem))

    mean_rewards_ttype23 = np.mean(ttype23_rewards)
    ttype23_rewards_sem = stats.sem(ttype23_rewards)
    if(print_debug == True):
        print("\n")
        print("ttype23")
        print(ttype23_rewards)
        print("mean: " + str(mean_rewards_ttype23))
        print("SEM: " + str(ttype23_rewards_sem))

    mean_rewards_ttype24 = np.mean(ttype24_rewards)
    ttype24_rewards_sem = stats.sem(ttype24_rewards)
    if(print_debug == True):    
        print("\n")
        print("ttype24")
        print(ttype24_rewards)
        print("mean: " + str(mean_rewards_ttype24))
        print("SEM: " + str(ttype24_rewards_sem))

    mean_rewards_ttype25 = np.mean(ttype25_rewards)
    ttype25_rewards_sem = stats.sem(ttype25_rewards)
    if(print_debug == True):
        print("\n")
        print("ttype25)")
        print(ttype25_rewards)
        print("mean: " + str(mean_rewards_ttype25))
        print("SEM: " + str(ttype25_rewards_sem))

    mean_rewards_ttype26 = np.mean(ttype26_rewards)    
    ttype26_rewards_sem = stats.sem(ttype26_rewards)
    if(print_debug == True):
        print("\n")
        print("ttype26)")
        print(ttype26_rewards)
        print("mean: " + str(mean_rewards_ttype26))
        print("SEM: " + str(ttype26_rewards_sem))

    ttype2_concat = np.concatenate((ttype23_rewards, ttype24_rewards, ttype25_rewards, ttype26_rewards), axis=None)
    mean_rewards_ttype2_concat = np.mean(ttype2_concat)
    ttype2_concat_sem = stats.sem(ttype2_concat)
    if(print_debug == True):
        print("\n")
        print("ttype2 concatenated")
        print(ttype2_concat)
        print("mean: " + str(mean_rewards_ttype2_concat))
        print("SEM: " + str(ttype2_concat_sem))
    
    mean_rewards_ttype47 = np.mean(ttype47_rewards)
    ttype47_rewards_std = np.std(ttype47_rewards)
    ttype47_rewards_sem = stats.sem(ttype47_rewards)
    if(print_debug == True):
        print("\n")
        print("ttype47)")
        print(ttype47_rewards)
        print("mean: " + str(mean_rewards_ttype47))
        print("SEM: " + str(ttype47_rewards_sem))
        print("std: " + str(ttype47_rewards_std))

    mean_rewards_ttype49 = np.mean(ttype49_rewards)
    ttype49_rewards_std = np.std(ttype49_rewards)
    ttype49_rewards_sem = stats.sem(ttype49_rewards)
    if(print_debug == True):
        print("\n")
        print("ttype49)")
        print(ttype49_rewards)
        print("mean: " + str(mean_rewards_ttype49))
        print("SEM: " + str(ttype49_rewards_sem))
        print("std: " + str(ttype49_rewards_std))

    ttype4_rewards_concat = np.concatenate((ttype47_rewards, ttype49_rewards), axis=None)
    mean_rewards_ttype4_concat = np.mean(ttype4_rewards_concat)
    ttype4_rewards_std_concat = np.std(ttype4_rewards_concat)
    ttype4_rewards_sem_concat = stats.sem(ttype4_rewards_concat)
    if(print_debug == True):
        print("\n")
        print("ttype4_average)")
        print(ttype4_rewards_concat)
        print("mean: " + str(mean_rewards_ttype4_concat))
        print("SEM: " + str(ttype4_rewards_sem_concat))
        print("std: " + str(ttype4_rewards_std_concat))

    mean_rewards_ttype5 = np.mean(ttype5_rewards)
    ttype5_rewards_std = np.std(ttype5_rewards)
    ttype5_rewards_sem = stats.sem(ttype5_rewards)
    if(print_debug == True):
        print("\n")
        print("ttype5)")
        print(ttype5_rewards)
        print("mean: " + str(mean_rewards_ttype5))
        print("SEM: " + str(ttype5_rewards_sem))
        print("std: " + str(ttype5_rewards_std))


    print("\n  ####### ANOVA results ############## \n \n ")
    print("One way ANOVA  : rewards among different task types ")
    one_way_anova_rewards = stats.f_oneway(ttype1_rewards, ttype23_rewards, ttype47_rewards)
    print(one_way_anova_rewards)
    

    # Calculate task duration
    print(" ANOVA task duration: ")
    one_way_anova_duration = stats.f_oneway(ttype1_task_duration, ttype2_task_duration, ttype4_task_duration)
    print(one_way_anova_duration)


    # Calculate number of robot take overs 
    print("\n robot take over anova: ")
    robot_take_anova = stats.f_oneway(ttype1_robot_takeover,ttype2_robot_takeover,ttype4_robot_takeover)
    print(robot_take_anova)

    ttype1_mean_task_duration = np.mean(ttype1_task_duration)
    ttype2_mean_task_duration = np.mean(ttype2_task_duration)
    ttype23_mean_task_duration = np.mean(ttype23_task_duration)
    ttype4_mean_task_duration = np.mean(ttype4_task_duration)
    ttype1_task_duration_sem = stats.sem(ttype1_task_duration)
    ttype23_task_duration_sem = stats.sem(ttype23_task_duration)
    ttype4_task_duration_sem = stats.sem(ttype4_task_duration)


    #results = ols('libido ~ C(dose)', data=df).fit()
    #results.summary()

    # ########################
    #
    #       + Plotting REWARDS + 
    #  
    # ########## create lists for plot ############################################################ 
    x_labels = ['typ1','typ2_first','type2_average', 'type4_first', 'type4_average']
    x_pos = np.arange(len(x_labels))
    reward_means_first = [mean_rewards_ttype1,mean_rewards_ttype23,mean_rewards_ttype47]
    reward_sem_first   = [ttype1_rewards_sem,ttype23_rewards_sem,ttype47_rewards_sem]
    reward_means_average = [mean_rewards_ttype2_concat,mean_rewards_ttype4_concat]
    reward_sem_average   = [ttype2_concat_sem,ttype4_rewards_sem_concat]


    # ######### plot and save ######################

# ###### rewards ###################################  
# 
 
    #Anova results for legend 
    p_value = '%1.5f' % float(one_way_anova_rewards[1])
    F_value = '%4.5f' % float(one_way_anova_rewards[0])
    plt.plot([], [], ' ', label=('F= ' + str(F_value) + ' p= ' + str(p_value)) )

    fig, ax = plt.subplots()
    first_idx = [0,1,3]
    bar_plot = ax.bar(x_pos[first_idx], reward_means_first, color='blue', yerr=reward_sem_first, align='center', alpha=0.5, ecolor='black', capsize=10, label='first occurrence + SEM bar' )
    average_idx = [2,4]
    bar_plot_average = ax.bar(x_pos[average_idx], reward_means_average, color='dodgerblue', yerr=reward_sem_average, align='center', alpha=0.5, ecolor='black', capsize=10, label='average + SEM bar' )
    plt.plot([], [], ' ', label=('F= ' + str(F_value) + ' p= ' + str(p_value)) )


    ax.set_ylabel('Rewards received by human robot team')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(x_labels)
    ax.set_xlabel('task types')
    ax.set_title('Rewards received during 3 different task types')
    ax.yaxis.grid(True)

        # label the bar with the value 
    for rect in bar_plot:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width()/6.0, 1.005*height,
                '%8.1f' % float(height),
                ha='center', va='bottom')

    for rect in bar_plot_average:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width()/6.0, 1.005*height,
                '%8.1f' % float(height),
                ha='center', va='bottom')

    # Save the figure and show

    plt.legend(loc='upper left')
    plt.tight_layout()
    plt.savefig(dir + '/../Challenging_task_plots/rewards_mean_sem.png')
    if(show_plots == True):
        plt.show()

# #######################
    # ###################  plotting rewards vs time taken #######################
    
    #  x_labels = ['typ1','type2_average', 'type4_average', 'type_5']
    #x_pos = np.arange(np.amax(ttype2_task_duration))
    print(len(x_pos))
    print(len(ttype2_rewards))
    print("duration" + str(ttype23_task_duration))
    print("rewards" + str(ttype23_rewards))
    reward_means = [mean_rewards_ttype1,mean_rewards_ttype23,mean_rewards_ttype2_concat,mean_rewards_ttype47,mean_rewards_ttype4_concat,mean_rewards_ttype5]
    reward_sem   = [ttype1_rewards_sem,ttype23_rewards_sem,ttype2_concat_sem,ttype47_rewards_sem,ttype4_rewards_sem_concat,ttype5_rewards_sem]

    # ######### plot and save ##################
    fig, ax = plt.subplots()

    # ######## calculate linear regression 

    # Generated linear fit
    slope, intercept, r_value, p_value, std_err = stats.linregress(ttype23_task_duration,ttype23_rewards)
    line2 = slope*ttype23_task_duration+intercept

    slope, intercept, r_value, p_value, std_err = stats.linregress(ttype47_task_duration,ttype47_rewards)
    line4 = slope*ttype47_task_duration+intercept

    ax.plot(ttype23_task_duration,ttype23_rewards,'o', c='b', label='type2')
    ax.plot(ttype23_task_duration,line2, c='b', label='regression type2')
    
    ax.plot(ttype47_task_duration,ttype47_rewards,'o',c='r', label='type4')
    ax.plot(ttype47_task_duration,line4, c='r', label='regression type4')

    #ax.scatter(ttype23_task_duration, ttype23_rewards, c='b', label='type2')
    #ax.scatter(ttype47_task_duration,ttype47_rewards, c='r', label='type4')
    ax.set_ylabel('Mean rewards received by human robot team')
    #ax.set_xticks(x_pos)
    #ax.set_xticklabels(x_labels)
    ax.set_xlabel('task duration')
    ax.set_title('Rewards received over task duration for 3 different task types')
    ax.yaxis.grid(True)
    plt.legend(loc='upper left')

    # Save the figure and show
    plt.tight_layout()
    plt.savefig(dir + '/../Challenging_task_plots/rewards_vs_time.png')
    if(show_plots == True):
        plt.show()

# ###################################
# Plotting task duration means per task type 

    p_value = '%1.5f' % float(one_way_anova_duration[1])
    F_value = '%4.5f' % float(one_way_anova_duration[0])
    

    x_labels = ['typ1','typ2','type4']
    x_pos = np.arange(len(x_labels))
    y_data = [ttype1_mean_task_duration ,ttype23_mean_task_duration,ttype4_mean_task_duration]
    y_sem  = [ttype1_task_duration_sem,ttype23_task_duration_sem,ttype4_task_duration_sem]

    # ######### plot and save ##################
    fig, ax = plt.subplots()
    duration_plot = ax.bar(x_pos, y_data, color='blue', yerr=y_sem, align='center', alpha=0.5, ecolor='black', capsize=10,label='mean + SEM bar')
    plt.plot([], [], ' ', label=('F= ' + str(F_value) + ' p= ' + str(p_value)) )
    ax.set_ylabel('Mean task duration [seconds]')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(x_labels)
    ax.set_xlabel('task types')
    ax.set_title('Mean task duration of 3 different task types')
    ax.yaxis.grid(True)

    # label the bar with the value 
    for rect in duration_plot:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width()/3.7, 1.005*height,
                '%8.1f' % float(height),
                ha='center', va='bottom')

    # Save the figure and show
    plt.legend(loc='upper left')
    plt.tight_layout()
    plt.savefig(dir + '/../Challenging_task_plots/task_duration_mean_sem.png')
    #if(show_plots == True):
    plt.show()     




# ###################################
# Plotting robot interference per task 

     #Anova results for legend 
    p_value = '%1.5f' % float(robot_take_anova[1])
    F_value = '%4.5f' % float(robot_take_anova[0])

    x_labels = ['typ1','typ2','type4']
    x_pos = np.arange(len(x_labels))
    y_data = [mean_robot_takeover_ttype1,mean_robot_takeover_ttype2,mean_robot_takeover_ttype4]
    y_sem  = [sem_robot_takeover_ttype1,sem_robot_takeover_ttype2,sem_robot_takeover_ttype4]

    # ######### plot and save ##################
    fig, ax = plt.subplots()
    duration_plot = ax.bar(x_pos, y_data, color='blue', yerr=y_sem, align='center', alpha=0.5, ecolor='black', capsize=10,label='mean + SEM bar')
    plt.plot([], [], ' ', label=('F= ' + str(F_value) + ' p= ' + str(p_value)) )
    ax.set_ylabel('Mean number of robot interventions')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(x_labels)
    ax.set_xlabel('task types')
    ax.set_title('Robot interventions during 3 different task types')
    ax.yaxis.grid(True)

    # label the bar with the value 
    for rect in duration_plot:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width()/3.7, 1.005*height,
                '%8.1f' % float(height),
                ha='center', va='bottom')

    # Save the figure and show
    plt.legend(loc='upper left')
    plt.tight_layout()
    plt.savefig(dir + '/../Challenging_task_plots/robot_interventions_mean_sem.png')
    #if(show_plots == True):
    plt.show()    



# helper functions for analysis 


# I have to make sure manually that each task has a sensor reading reporting the success
def arrange_human_obs(dir):
    with open(dir + '/human_observables.csv', 'rb') as file:
        reader = csv.reader(file, delimiter=',')
        human_obs = list(reader)

    for col_index in range(len(human_obs[0])):
        if human_obs[0][col_index] == "task_id":
            col_taskID = col_index
        elif human_obs[0][col_index] == "human_model":
            col_model = col_index

    with open(dir + '/human_observables_final.csv', 'a') as file:
        wr = csv.writer(file, dialect='excel')
        first_row = human_obs[0]
        wr.writerow(first_row)

        dis_col = []
        dis_noncol = []
        exp_ntc = []
        exp_ntnc = []
        exp_tc = []
        exp_tnc = []
        beg_ntc = []
        beg_ntnc = []
        beg_tc = []
        beg_tnc = []
        new_row = []

        for row_index in range(len(human_obs)):
            if row_index == 0:  # skip the first row which has titles only
                continue
            if human_obs[row_index][col_model] == '"distracted_collaborative.POMDPx"':
                dis_col.append(human_obs[row_index])
            if human_obs[row_index][col_model] == '"distracted_noncollaborative.POMDPx"':
                dis_noncol.append(human_obs[row_index])
            if human_obs[row_index][col_model] == '"expert_nontired_collaborative.POMDPx"':
                exp_ntc.append(human_obs[row_index])
            if human_obs[row_index][col_model] == '"expert_nontired_noncollaborative.POMDPx"':
                exp_ntnc.append(human_obs[row_index])
            if human_obs[row_index][col_model] == '"expert_tired_collaborative.POMDPx"':
                exp_tc.append(human_obs[row_index])
            if human_obs[row_index][col_model] == '"expert_tired_noncollaborative.POMDPx"':
                exp_tnc.append(human_obs[row_index])
            if human_obs[row_index][col_model] == '"beginner_nontired_collaborative.POMDPx"':
                beg_ntc.append(human_obs[row_index])
            if human_obs[row_index][col_model] == '"beginner_nontired_noncollaborative.POMDPx"':
                beg_ntnc.append(human_obs[row_index])
            if human_obs[row_index][col_model] == '"beginner_tired_collaborative.POMDPx"':
                beg_tc.append(human_obs[row_index])
            if human_obs[row_index][col_model] == '"beginner_tired_noncollaborative.POMDPx"':
                beg_tnc.append(human_obs[row_index])

        new_lines = np.concatenate([dis_col, dis_noncol, exp_ntc, exp_ntnc, exp_tc,
                                exp_tnc, beg_ntc, beg_ntnc, beg_tc, beg_tnc])
        #new_row.append(dis_noncol)
        #new_row.append(exp_ntc)
        #new_row.append(exp_ntnc)
        #new_row.append(exp_tc)
        #new_row.append(exp_tnc)
        #new_row.append(beg_ntc)
        #new_row.append(beg_ntnc)
        #new_row.append(beg_tc)
        #new_row.append(beg_tnc)
        for row in range(len(new_lines)):
            wr.writerow(new_lines[row])

    file.close()
    print('INFO: human observables are saved separately under the folder provided')

if __name__ == '__main__':
   # work_book = open(sys.argv[1] + '/raw_data.csv', "rb")
   # print('Reading:' + sys.argv[1] + '/raw_data.csv')
   # reader = csv.reader(work_book, delimiter=',')
   # raw = list(reader)

    if sys.argv[2] == "save_all":
        save_sensor_data(sys.argv[1], raw)
        human_observables(sys.argv[1], raw)
        arrange_human_obs(sys.argv[1])
        task_status(sys.argv[1])
    if sys.argv[2] == "save_sensors":
        save_sensor_data(sys.argv[1], raw)
    if sys.argv[2] == "save_human_obs":
        human_observables(sys.argv[1], raw)
    if sys.argv[2] == "arrange_human_obs":
        arrange_human_obs(sys.argv[1])
    if sys.argv[2] == "task_status":
        task_status(sys.argv[1])
    if sys.argv[2] == "plot_challenging":
        plot_challenging(sys.argv[1],sys.argv[3],sys.argv[4])
    if sys.argv[2] == "filter_csvs":
        filter_csvs(sys.argv[1])


   # work_book.close()
