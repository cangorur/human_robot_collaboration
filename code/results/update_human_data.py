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
            if raw[row_index][col_reporter] == "HUMAN":
                wr.writerow(raw[row_index])
    file.close()


def append_human_states(dir):
    with open(dir + '/human_update.csv', 'rb') as file:
        reader = csv.reader(file, delimiter=',')
        human_updates = list(reader)

        for col_index in range(len(human_updates[0])):
            if human_updates[0][col_index] == "task_id":
                col_taskID = col_index
            elif human_updates[0][col_index] == "secs":
                col_secs = col_index
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
                    if human_updates[row_index][col_states] == "TaskHuman":
                        new_row[col_states] = human_updates[row_index + 1][col_states]
                    wr.writerow(new_row)
                    seconds += 1
        new_file.close()
    file.close()


def human_state_distr(dir):
    with open(dir + '/appended_human_updates.csv', 'rb') as file:
        reader = csv.reader(file, delimiter=',')
        human_updates = list(reader)

        for col_index in range(len(human_updates[0])):
            if human_updates[0][col_index] == "task_id":
                col_taskID = col_index
            elif human_updates[0][col_index] == "human_expertise":
                col_exp = col_index
            elif human_updates[0][col_index] == "human_mood":
                col_mood = col_index
            elif human_updates[0][col_index] == "human_trust":
                col_trust= col_index
            elif human_updates[0][col_index] == "belief_state":
                col_state = col_index

        with open(dir + '/human_state_distr.csv', 'a') as new_file:
            wr = csv.writer(new_file, dialect='excel')
            # first row:
            first_row = ["task_id", "expertise", "mood", "trust", "TaskHuman", "GlobalSuccess", "GlobalFail", "FailedToGrasp",
                         "NoAttention", "Evaluating", "Tired", "Recovery", "WarningTheRobot", "RobotIsWarned",
                         "TaskRobot"]
            wr.writerow(first_row)
            new_row = [None] * 15  # initiate a row
            task_id = 1
            s0_ctr = 0  # every count is one second
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

            for row_index in range(len(human_updates)):

                if row_index == 0:  # skip the first row which has titles only
                    continue
                human_state = human_updates[row_index][col_state]
                if human_state == "TaskHuman":
                    s0_ctr += 1
                elif human_state == "GlobalSuccess":
                    s1_ctr += 1
                elif human_state == "GlobalFail":
                    s2_ctr += 1
                elif human_state == "FailedToGrasp":
                    s3_ctr += 1
                elif human_state == "NoAttention":
                    s4_ctr += 1
                elif human_state == "Evaluating":
                    s5_ctr += 1
                elif human_state == "Tired":
                    s6_ctr += 1
                elif human_state == "Recovery":
                    s7_ctr += 1
                elif human_state == "WarningTheRobot":
                    s8_ctr += 1
                elif human_state == "RobotIsWarned":
                    s9_ctr += 1
                elif human_state == "TaskRobot":
                    s10_ctr += 1

                if row_index+1 == len(human_updates) or int(human_updates[row_index+1][col_taskID]) != task_id:
                    # information on this task_id has finished
                    new_row[0] = human_updates[row_index][col_taskID]
                    new_row[1] = human_updates[row_index][col_exp]
                    new_row[2] = human_updates[row_index][col_mood]
                    new_row[3] = human_updates[row_index][col_trust]
                    new_row[4] = str(s0_ctr)
                    new_row[5] = str(s1_ctr)
                    new_row[6] = str(s2_ctr)
                    new_row[7] = str(s3_ctr)
                    new_row[8] = str(s4_ctr)
                    new_row[9] = str(s5_ctr)
                    new_row[10] = str(s6_ctr)
                    new_row[11] = str(s7_ctr)
                    new_row[12] = str(s8_ctr)
                    new_row[13] = str(s9_ctr)
                    new_row[14] = str(s10_ctr)

                    wr.writerow(new_row)  # finished a row for a task
                    task_id += 1
                    new_row = [None] * 15  # initiate a row
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
                    # make the counters zero

        new_file.close()
    file.close()

if __name__ == '__main__':
    work_book = open(sys.argv[1] + '/raw_data.csv', "rb")
    reader = csv.reader(work_book, delimiter=',')
    raw = list(reader)

    if sys.argv[2] == "save_human":
        save_human_data(sys.argv[1], raw)
    if sys.argv[2] == "append_human":
        append_human_states(sys.argv[1])
    if sys.argv[2] == "state_distribution":
        human_state_distr(sys.argv[1])
