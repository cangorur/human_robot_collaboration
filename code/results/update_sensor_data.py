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
            if raw[row_index][col_reporter] == "MANAGER":
                wr.writerow(raw[row_index])
            elif raw[row_index][col_reporter] == "SENSORS" and \
                            raw[row_index][col_taskSt] != "ONGOING":
                wr.writerow(raw[row_index])
    file.close()


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
                if sensor_updates[row_index][col_whoRep] == "MANAGER":
                    new_row[0] = sensor_updates[row_index][col_taskID]
                    new_row[1] = sensor_updates[row_index][col_update_time + 1]
                    new_row[2] = sensor_updates[row_index][col_update_time + 2]
                    manager_flag = True
                if sensor_updates[row_index][col_whoRep] == "SENSORS" and \
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


if __name__ == '__main__':
    work_book = open(sys.argv[1] + '/raw_data.csv', "rb")
    reader = csv.reader(work_book, delimiter=',')
    raw = list(reader)

    if sys.argv[2] == "save_sensors":
        save_sensor_data(sys.argv[1], raw)
    if sys.argv[2] == "task_status":
        task_status(sys.argv[1])
