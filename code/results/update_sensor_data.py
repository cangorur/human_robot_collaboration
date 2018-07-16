import csv
import sys


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
            elif raw[row_index][col_reporter] == '"SENSORS"' and \
                            raw[row_index][col_taskSt] != '"ONGOING"':
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
        elif raw[0][col_index] == "human_observables":
            col_humanObs = col_index

    with open(dir + '/human_observables.csv', 'a') as file:
        wr = csv.writer(file, dialect='excel')
        first_row = ["task_id", "step_count", "who_reports", "update_received_time", "update_secs", "update_nsecs", "human_observables",
                    "Human Detected", "Looked Around", "Succesfull Grasp", "Failed Grasp", "Warned Robot", "Stood Idle"]

        wr.writerow(first_row)
        new_row = [None] * 13  # initiate a row

        for row_index in range(len(raw)):
            if row_index == 0:  # skip the first row which has titles only
                continue
            if raw[row_index][col_whoRep] == '"OBSERVATION"':
                new_row[0] = raw[row_index][col_taskID]
                new_row[1] = raw[row_index][col_step]
                new_row[2] = raw[row_index][col_whoRep]
                new_row[3] = raw[row_index][col_update_time]
                new_row[4] = raw[row_index][col_update_time + 1]
                new_row[5] = raw[row_index][col_update_time + 2]
                new_row[6] = raw[row_index][col_humanObs]
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

                new_row[7] = det
                new_row[8] = la
                new_row[9] = sucGra
                new_row[10] = failGra
                new_row[11] = warn
                new_row[12] = idle

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
            elif sensor_updates[0][col_index] == "who_succeeded":
                col_whoSuc = col_index

        with open(dir + '/task_status.csv', 'a') as new_file:
            wr = csv.writer(new_file, dialect='excel')
            # first row:
            first_row = ["task_id", "init_secs", "init_nsecs", "fin_secs", "fin_nsecs", "time_took", "task_status",
                         "who_succeeded"]
            wr.writerow(first_row)
            new_row = [None] * 8  # initiate a row
            manager_flag = False
            finish_report_flag = False
            for row_index in range(len(sensor_updates)):

                if row_index == 0:  # skip the first row which has titles only
                    continue
                if sensor_updates[row_index][col_whoRep] == '"MANAGER"':
                    new_row[0] = sensor_updates[row_index][col_taskID]
                    new_row[1] = sensor_updates[row_index][col_update_time + 1]
                    new_row[2] = sensor_updates[row_index][col_update_time + 2]
                    manager_flag = True
                if sensor_updates[row_index][col_whoRep] == '"SENSORS"' and \
                                new_row[0] == sensor_updates[row_index][
                            col_taskID]:  # if the same task with manager report
                    new_row[3] = sensor_updates[row_index][col_update_time + 1]
                    new_row[4] = sensor_updates[row_index][col_update_time + 2]
                    init_time = new_row[1] + '.' + new_row[2]
                    fin_time = new_row[3] + '.' + new_row[4]
                    new_row[5] = float(fin_time) - float(init_time)
                    new_row[6] = sensor_updates[row_index][col_taskSt]
                    new_row[7] = sensor_updates[row_index][col_whoSuc]
                    finish_report_flag = True

                if manager_flag and finish_report_flag:
                    wr.writerow(new_row)  # finished a row for a task
                    new_row = [None] * 8  # initiate a row
                    manager_flag = False
                    finish_report_flag = False

        new_file.close()
    file.close()
    print 'INFO: task status information is saved separately under the folder provided'

if __name__ == '__main__':
    work_book = open(sys.argv[1] + '/raw_data.csv', "rb")
    print 'Reading:' + sys.argv[1] + '/raw_data.csv'
    reader = csv.reader(work_book, delimiter=',')
    raw = list(reader)

    if sys.argv[2] == "save_sensors":
        save_sensor_data(sys.argv[1], raw)
    if sys.argv[2] == "save_human_obs":
        human_observables(sys.argv[1], raw)
    if sys.argv[2] == "task_status":
        task_status(sys.argv[1])

    work_book.close()
