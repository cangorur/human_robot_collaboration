import csv
import sys


def save_human_data(dir, raw):
    for col_index in range(len(raw[0])):
        if raw[0][col_index] == "who_reports":
            col_reporter = col_index
            break

    with open(dir + '/human_update.csv', 'a') as file:
        wr = csv.writer(file, dialect='excel')
        wr.writerow(raw[0])

        for row_index in range(len(raw)):
            if raw[row_index][col_reporter] == '"HUMAN"':
                wr.writerow(raw[row_index])
    file.close()
    print 'INFO: human information is saved separately under the folder provided'


def append_human_states(dir):
    with open(dir + '/human_update.csv', 'rb') as file:
        reader = csv.reader(file, delimiter=',')
        human_updates = list(reader)

        for col_index in range(len(human_updates[0])):
            if human_updates[0][col_index] == "task_id":
                col_taskID = col_index
            elif human_updates[0][col_index] == "action_taken_time":
                col_secs = col_index + 1
            elif human_updates[0][col_index] == "belief_state":
                col_states = col_index

        with open(dir + '/appended_human_updates.csv', 'a') as new_file:
            wr = csv.writer(new_file, dialect='excel')
            wr.writerow(human_updates[0])
            for row_index in range(len(human_updates)):
                if row_index == 0:
                    continue
                elif row_index + 1 == len(human_updates):
                    wr.writerow(human_updates[row_index])
                    continue
                wr.writerow(human_updates[row_index])
                seconds = int(human_updates[row_index][col_secs])
                task_id = human_updates[row_index][col_taskID]
                while seconds + 1 < int(human_updates[row_index + 1][col_secs]) \
                        and task_id == human_updates[row_index + 1][col_taskID]:
                    new_row = human_updates[row_index]
                    new_row[col_secs] = str(seconds + 1)
                    if human_updates[row_index][col_states] == '"TaskHuman"':
                        new_row[col_states] = human_updates[row_index + 1][col_states]
                    wr.writerow(new_row)
                    seconds += 1
        new_file.close()
    file.close()
    print 'INFO: human information has been appended'


def human_state_distr(dir):
    with open(dir + '/appended_human_updates.csv', 'rb') as file:
        reader = csv.reader(file, delimiter=',')
        human_updates = list(reader)

        for col_index in range(len(human_updates[0])):
            if human_updates[0][col_index] == "task_id":
                col_taskID = col_index
            elif human_updates[0][col_index] == "human_model":
                col_model = col_index
            elif human_updates[0][col_index] == "belief_state":
                col_state = col_index

        with open(dir + '/human_state_distr.csv', 'a') as new_file:
            wr = csv.writer(new_file, dialect='excel')
            # first row:
            first_row = ["task_id", "type", "TaskHuman", "GlobalSuccess", "GlobalFail", "FailedToGrasp",
                         "NoAttention", "Evaluating", "Tired", "Recovery", "RobotIntefered", "WarningTheRobot", "RobotIsWarned",
                         "TaskRobot", "TaskHuman_avg", "TaskHuman_avg", "GlobalSuccess_avg", "GlobalFail_avg", "FailedToGrasp_avg",
                         "NoAttention_avg", "Evaluating_avg", "Tired_avg", "Recovery_avg", "RobotIntefered_avg", "WarningTheRobot_avg",
                         "RobotIsWarned_avg", "TaskRobot_avg"]
            wr.writerow(first_row)
            new_row = [None] * 26  # initiate a row
            task_id = int(human_updates[1][col_taskID])
            task_per_type = task_id
            [s0_ctr, s1_ctr, s2_ctr, s3_ctr, s4_ctr, s5_ctr, s6_ctr, s7_ctr,
                s8_ctr, s9_ctr, s10_ctr, s11_ctr]= [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # every count is one second

            # cumulative states to calculate moving average
            [cum_s0, cum_s1, cum_s2, cum_s3, cum_s4, cum_s5, cum_s6, cum_s7, cum_s8,
                cum_s9, cum_s10, cum_s11] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            for row_index in range(len(human_updates)):

                if row_index == 0:  # skip the first row which has titles only
                    continue
                human_state = human_updates[row_index][col_state]
                if human_state == '"TaskHuman"':
                    s0_ctr += 1
                elif human_state == '"GlobalSuccess"':
                    s1_ctr += 1
                elif human_state == '"GlobalFail"':
                    s2_ctr += 1
                elif human_state == '"FailedToGrasp"':
                    s3_ctr += 1
                elif human_state == '"NoAttention"':
                    s4_ctr += 1
                elif human_state == '"Evaluating"':
                    s5_ctr += 1
                elif human_state == '"Tired"':
                    s6_ctr += 1
                elif human_state == '"Recovery"':
                    s7_ctr += 1
                elif human_state == '"RobotInterfered"':
                    s8_ctr += 1
                elif human_state == '"WarningTheRobot"':
                    s9_ctr += 1
                elif human_state == '"RobotIsWarned"':
                    s10_ctr += 1
                elif human_state == '"TaskRobot"':
                    s11_ctr += 1

                if row_index+1 == len(human_updates) or int(human_updates[row_index+1][col_taskID]) != task_id:
                    # information on this task_id has finished
                    new_row[0] = human_updates[row_index][col_taskID]
                    new_row[1] = human_updates[row_index][col_model]
                    new_row[2] = str(s0_ctr)
                    new_row[3] = str(s1_ctr)
                    new_row[4] = str(s2_ctr)
                    new_row[5] = str(s3_ctr)
                    new_row[6] = str(s4_ctr)
                    new_row[7] = str(s5_ctr)
                    new_row[8] = str(s6_ctr)
                    new_row[9] = str(s7_ctr)
                    new_row[10] = str(s8_ctr)
                    new_row[11] = str(s9_ctr)
                    new_row[12] = str(s10_ctr)
                    new_row[13] = str(s11_ctr)

                    cum_s0 +=  s0_ctr
                    cum_s1 +=  s1_ctr
                    cum_s2 +=  s2_ctr
                    cum_s3 +=  s3_ctr
                    cum_s4 +=  s4_ctr
                    cum_s5 +=  s5_ctr
                    cum_s6 +=  s6_ctr
                    cum_s7 +=  s7_ctr
                    cum_s8 +=  s8_ctr
                    cum_s9 +=  s9_ctr
                    cum_s10 +=  s10_ctr
                    cum_s11 +=  s11_ctr

                    new_row[14] = float(cum_s0 / task_per_type)
                    new_row[15] = float(cum_s1 / task_per_type)
                    new_row[16] = float(cum_s2 / task_per_type)
                    new_row[17] = float(cum_s3 / task_per_type)
                    new_row[18] = float(cum_s4 / task_per_type)
                    new_row[19] = float(cum_s5 / task_per_type)
                    new_row[20] = float(cum_s6 / task_per_type)
                    new_row[21] = float(cum_s7 / task_per_type)
                    new_row[22] = float(cum_s8 / task_per_type)
                    new_row[23] = float(cum_s9 / task_per_type)
                    new_row[24] = float(cum_s10 / task_per_type)
                    new_row[25] = float(cum_s11 / task_per_type)

                    wr.writerow(new_row)  # finished a row for a task
                    task_id += 1
                    task_per_type += 1
                    new_row = [None] * 26  # initiate a row
                    s0_ctr = 0
                    s1_ctr = 0
                    s2_ctr = 0
                    s3_ctr = 0
                    s4_ctr = 0
                    s5_ctr = 0
                    s6_ctr = 0
                    s7_ctr = 0
                    s8_ctr = 0
                    s9_ctr = 0
                    s10_ctr = 0
                    s11_ctr = 0
                    # make the cumulated values zero when type changes, i.e. a new averaging
                    if row_index+1 < len(human_updates) and human_updates[row_index][col_model] != human_updates[row_index+1][col_model]:
                        task_per_type = 1
                        cum_s0 = 0.0
                        cum_s1 = 0.0
                        cum_s2 = 0.0
                        cum_s3 = 0.0
                        cum_s4 = 0.0
                        cum_s5 = 0.0
                        cum_s6 = 0.0
                        cum_s7 = 0.0
                        cum_s8 = 0.0
                        cum_s9 = 0.0
                        cum_s10 = 0.0
                        cum_s11 = 0.0


        new_file.close()
    file.close()
    print 'INFO: human state distribution has been provided'

if __name__ == '__main__':
    work_book = open(sys.argv[1] + '/raw_data.csv', "rb")
    reader = csv.reader(work_book, delimiter=',')
    raw = list(reader)

    if sys.argv[2] == "save_all":
        save_human_data(sys.argv[1], raw)
        append_human_states(sys.argv[1])
        human_state_distr(sys.argv[1])
    if sys.argv[2] == "save_human":
        save_human_data(sys.argv[1], raw)
    if sys.argv[2] == "append_human":
        append_human_states(sys.argv[1])
    if sys.argv[2] == "state_distribution":
        human_state_distr(sys.argv[1])

    work_book.close()
