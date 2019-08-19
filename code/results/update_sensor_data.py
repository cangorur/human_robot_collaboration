import csv
import sys
import numpy as np


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
            elif raw[row_index][col_reporter] == '"MANAGER-TASK-DONE"': #and raw[row_index][col_taskSt] != '"ONGOING"':
                wr.writerow(raw[row_index])
            elif raw[row_index][col_reporter] == '"SENSOR"': #and raw[row_index][col_taskSt] != '"ONGOING"':
                wr.writerow(raw[row_index])
    file.close()
    print 'INFO: sensor information is saved separately under the folder provided'

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
    print 'INFO: human observables are saved separately under the folder provided'


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
            elif sensor_updates[0][col_index] == "task_status":
                col_taskSt = col_index
            elif sensor_updates[0][col_index] == "who_succeeded_subtask":
                col_whoSuc = col_index

        with open(dir + '/task_status.csv', 'a') as new_file:
            wr = csv.writer(new_file, dialect='excel')
            # first row:
            first_row = ["task_id", "init_secs", "init_nsecs", "fin_secs", "fin_nsecs", "time_took", "moving_avg_time",
            "task_status", "moving_avg_status", "who_succeeded"]
            wr.writerow(first_row)
            new_row = [None] * 10  # initiate a row
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
                if sensor_updates[row_index][col_whoRep] == '"MANAGER-TASK-DONE"' and \
                                new_row[0] == sensor_updates[row_index][
                            col_taskID]:  # if the same task with manager report
                    new_row[3] = sensor_updates[row_index][col_update_time + 1]
                    new_row[4] = sensor_updates[row_index][col_update_time + 2]
                    init_time = new_row[1] + '.' + new_row[2]
                    fin_time = new_row[3] + '.' + new_row[4]
                    new_row[5] = float(fin_time) - float(init_time)
                    cumulative_timeTook = cumulative_timeTook + float(fin_time) - float(init_time)
                    new_row[6] = float(cumulative_timeTook / int(sensor_updates[row_index][col_taskID])) # moving avg time
                    if sensor_updates[row_index][col_taskSt] == '"success"':
                        new_row[7] = 1
                    if sensor_updates[row_index][col_taskSt] == '"fail"':
                        new_row[7] = 0
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
    print 'INFO: task status information is saved separately under the folder provided'

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
    print 'INFO: human observables are saved separately under the folder provided'

if __name__ == '__main__':
    work_book = open(sys.argv[1] + '/raw_data.csv', "rb")
    print 'Reading:' + sys.argv[1] + '/raw_data.csv'
    reader = csv.reader(work_book, delimiter=',')
    raw = list(reader)

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

    work_book.close()
