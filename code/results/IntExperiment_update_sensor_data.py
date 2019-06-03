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


# ##########################################################################################################################
# This function plots the characterstics of the challenging tasks py parsing a number of csv files and averaging over them 
# Use like this python IntExperiment_update_sensor_data.py ./Participants/Analysis/task_success_files plot_challenging

def plot_challenging(dir):

    file_list = getFileList(dir)
    print("\n")
    print(file_list)
    print("\n")
    
    # variables for analysis   ttype = tasktype 

    ttype1_rewards = np.array([], dtype=np.float64)
    ttype2_rewards = np.array([], dtype=np.float64)
    ttype24_rewards = np.array([], dtype=np.float64)
    ttype25_rewards = np.array([], dtype=np.float64)
    ttype3_rewards = np.array([], dtype=np.float64)
    ttype1_percentage_correct_subt = np.array([], dtype=np.float64)
    ttype1_robot_tookover = np.array([], dtype=np.float64)
    ttype1_task_duration = np.array([], dtype=np.float64)
    

    for result_file in file_list: 
        types_dict = {'rosbagTimestamp':str, 'task_id':int, 'subtask_id':int,	'step_count':int, 'who_reports':str, 'update_received_time':str,	'secs1':int,	'nsecs1':int, 'human_model':str,	'action_taken_time':str , 'secs':int , 'nsecs':int, 'taken_action':str, 'belief_state':str, 'real_state':str, 'isEstimationCorrect':str,	'warnings_count_subtask':int, 'real_obs_received':str, 'real_obs_received_array':str, 'obs_with_noise':str,'human_observables':str, 'mapped_observation_pomdp':int,	'mapped_observation_raw':int, 'robot_model':str, 'immediate_reward':str, 'total_disc_reward':str,'robot_belief':str,	'tray_update_received_time':str, 'secs':int,	'nsecs':int, 'subtask_status':str,	'who_succeeded_subtask':str,	'subtask_duration':float,	'failed_subtasks':int, 'successful_subtasks':int, 'percentage_successful_subtasks':float,	'task_status':str,	'task_duration':float,	'who_succeeded_task':str,	'warnings_count_task':int,	'successful_tasks_cnt':int,	'failed_tasks_cnt':int,	'percentage_successful_tasks':float}
        #file_frame = pd.read_csv(result_file,dtype = types_dict, names=['rosbagTimestamp', 'task_id', 'subtask_id',	'step_count', 'who_reports', 'update_received_time',	'secs1',	'nsecs1', 'human_model',	'action_taken_time', 'secs', 'nsecs', 'taken_action', 'belief_state', 'real_state', 'isEstimationCorrect',	'warnings_count_subtask', 'real_obs_received', 'real_obs_received_array', 'obs_with_noise','human_observables', 'mapped_observation_pomdp',	'mapped_observation_raw', 'robot_model', 'immediate_reward', 'total_disc_reward','robot_belief',	'tray_update_received_time', 'secs3',	'nsecs3', 'subtask_status',	'who_succeeded_subtask',	'subtask_duration',	'failed_subtasks', 'successful_subtasks', 'percentage_successful_subtasks',	'task_status',	'task_duration',	'who_succeeded_task',	'warnings_count_task',	'successful_tasks_cnt',	'failed_tasks_cnt',	'percentage_successful_tasks'])

        file_frame = pd.read_csv(result_file, dtype = types_dict)
        #print(file_frame)

        print(result_file)

        for row in (file_frame.index):

            # turn 2 | task type 1 -> means analysis + SEM for each participant  | additional " in the string need to be trimmed  
            if ( ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == 2 ) ): 
                ttype1_rewards = np.append( ttype1_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )

            # turn 3 | task type 2 -> means analysis + SEM for each participant 
            if ( ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == 3 ) ): 
                ttype2_rewards = np.append( ttype2_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )

            # turn 4 | task type 2 -> means analysis + SEM for each participant 
            if ( ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == 4 ) ): 
                ttype24_rewards = np.append( ttype24_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )

            # turn 5 | task type 2 -> means analysis + SEM for each participant 
            if ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == 5 ): 
                ttype25_rewards = np.append( ttype25_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )

            # turn 3 | task type 2 -> means analysis + SEM for each participant 
            if ((file_frame['who_reports'][row]) == '"MANAGER-TASK-DONE"') and (file_frame['task_id'][row] == 7 ): 
                ttype3_rewards = np.append( ttype3_rewards, float((file_frame['total_disc_reward'][row]).replace('"','')) )
    
    print("ttype1")
    ttype1_rewards_sem = stats.sem(ttype1_rewards)
    print(ttype1_rewards.dtype)  
    mean_rewards_ttype1 = np.mean(ttype1_rewards)
    print(ttype1_rewards)
    print(mean_rewards_ttype1)

    print(ttype1_rewards_sem)

    
    print("\n")
    print("ttype2")
    print(ttype2_rewards)
    mean_rewards_ttype2 = np.mean(ttype2_rewards)
    print(mean_rewards_ttype2)
    ttype2_rewards_sem = stats.sem(ttype2_rewards)
    print(ttype2_rewards_sem)

    print("\n")
    print("ttype24")
    print(ttype24_rewards)
    mean_rewards_ttype24 = np.mean(ttype24_rewards)
    print(mean_rewards_ttype24)
    ttype24_rewards_sem = stats.sem(ttype24_rewards)
    print(ttype24_rewards_sem)

    print("\n")
    print("ttype25)")
    print(ttype25_rewards)
    mean_rewards_ttype25 = np.mean(ttype25_rewards)
    print(mean_rewards_ttype25)
    ttype25_rewards_sem = stats.sem(ttype25_rewards)
    print(ttype25_rewards_sem)

    
    print("\n")
    print("ttype3)")
    print(ttype3_rewards)
    mean_rewards_ttype3 = np.mean(ttype3_rewards)
    print(mean_rewards_ttype3)
    ttype3_rewards_sem = stats.sem(ttype3_rewards)
    print(ttype3_rewards_sem)

    

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
        plot_challenging(sys.argv[1])


   # work_book.close()
