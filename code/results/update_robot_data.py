import csv
import sys


def save_robot_data(dir, raw):
    for col_index in range(len(raw[0])):
        if raw[0][col_index] == "who_reports":
            col_reporter = col_index
        if raw[0][col_index] == "update_received_time":
            col_upd_time = col_index
        if raw[0][col_index] == "action_taken_time":
            col_act_time = col_index

    with open(dir + '/robot_update.csv', 'a') as file:
        wr = csv.writer(file, dialect='excel')
        first_row = raw[0]
        first_row[col_upd_time + 1] = "update_secs"
        first_row[col_upd_time + 2] = "update_nsecs"
        first_row[col_act_time + 1] = "acted_secs"
        first_row[col_act_time + 2] = "acted_nsecs"
        wr.writerow(first_row)

        for row_index in range(len(raw)):
            if raw[row_index][col_reporter] == "ROBOT":
                wr.writerow(raw[row_index])
    file.close()


def get_important_information(dir):
    robot_file = open(dir + '/robot_update.csv', 'rb')
    robot_reader = csv.reader(robot_file, delimiter=',')
    robot_updates = list(robot_reader)

    status_file = open(dir + '/task_status.csv', 'rb')
    task_reader = csv.reader(status_file, delimiter=',')
    task_updates = list(task_reader)

    human_file = open(dir + '/human_update.csv', 'rb')
    human_reader = csv.reader(human_file, delimiter=',')
    human_updates = list(human_reader)

    for col_index in range(len(task_updates[0])):
        if task_updates[0][col_index] == "task_id":
            col_taskInit_taskID = col_index
        elif task_updates[0][col_index] == "init_secs":
            col_taskInit_secs = col_index
        elif task_updates[0][col_index] == "init_nsecs":
            col_taskInit_nsecs = col_index

    for col_index in range(len(human_updates[0])):
        if human_updates[0][col_index] == "task_id":
            col_human_taskID = col_index
        elif human_updates[0][col_index] == "step_count":
            col_humanStep = col_index
        elif human_updates[0][col_index] == "human_mood":
            col_humanMood = col_index
        elif human_updates[0][col_index] == "human_trust":
            col_humanTrust = col_index
        elif human_updates[0][col_index] == "belief_state":
            col_humanBelief = col_index

    for col_index in range(len(robot_updates[0])):
        if robot_updates[0][col_index] == "task_id":
            col_taskID = col_index
        elif robot_updates[0][col_index] == "step_count":
            col_step = col_index
        elif robot_updates[0][col_index] == "acted_secs":
            col_act_secs = col_index
        elif robot_updates[0][col_index] == "acted_nsecs":
            col_act_nsecs = col_index
        elif robot_updates[0][col_index] == "taken_action":
            col_action = col_index
        elif robot_updates[0][col_index] == "belief_state":
            col_belief_st = col_index
        elif robot_updates[0][col_index] == "real_state":
            col_real_st = col_index
        elif robot_updates[0][col_index] == "isEstimationCorrect":
            col_est = col_index
        elif robot_updates[0][col_index] == "immediate_reward":
            col_reward = col_index
        elif robot_updates[0][col_index] == "total_disc_reward":
            col_tot_reward = col_index

    with open(dir + '/robot_important_info.csv', 'a') as new_file:
        wr = csv.writer(new_file, dialect='excel')
        first_row = ["task_id", "task_init_secs", "task_init_nsecs", "acted_secs", "acted_nsecs", "taken_action",
                     "belief_state", "real_state", "human_state", "human_mood", "human_trust", "isEstimationCorrect", "immediate_reward",
                     "total_disc_reward", "warning_received"]
        wr.writerow(first_row)
        new_row = [None] * 15  # initiate a row
        task_id = 1
        is_important_state = False
        for row_index in range(len(robot_updates)):

            if row_index == 0 or row_index == 1:  # skip the first row which has titles only
                continue

            task_id = int(robot_updates[row_index][col_taskID])

            for row_task in range(len(task_updates)):
                if row_task == 0:  # skip the first row which has titles only
                    continue
                if int(task_updates[row_task][col_taskInit_taskID]) == task_id:
                    new_row[0] = task_updates[row_task][col_taskInit_taskID]
                    new_row[1] = task_updates[row_task][col_taskInit_secs]
                    new_row[2] = task_updates[row_task][col_taskInit_nsecs]
                    break

            new_row[3] = robot_updates[row_index][col_act_secs]
            new_row[4] = robot_updates[row_index][col_act_nsecs]
            new_row[5] = robot_updates[row_index][col_action]
            new_row[6] = robot_updates[row_index][col_belief_st]
            new_row[7] = robot_updates[row_index][col_real_st]
            new_row[11] = robot_updates[row_index][col_est]
            new_row[12] = robot_updates[row_index][col_reward]
            new_row[13] = robot_updates[row_index][col_tot_reward]
            if int(robot_updates[row_index][col_reward]) != 0 and \
                    robot_updates[row_index][col_real_st] == "WarningReceived":
                new_row[14] = "1"
            else:
                new_row[14] = "0"

            step_count = int(human_updates[row_index][col_step])
            for row_human in range(len(human_updates)):
                if row_human == 0:  # skip the first row which has titles only
                    continue
                if int(human_updates[row_human][col_human_taskID]) == task_id and \
                                int(human_updates[row_human][col_humanStep]) == step_count:
                    new_row[8] = human_updates[row_human][col_humanBelief]
                    new_row[9] = human_updates[row_human][col_humanMood]
                    new_row[10] = human_updates[row_human][col_humanTrust]
                    break
                else:
                    new_row[8] = ""
                    new_row[9] = ""
                    new_row[10] = ""

            wr.writerow(new_row)  # finished a row for a task
            new_row = [None] * 15  # initiate a row

    new_file.close()
    human_file.close()
    status_file.close()
    robot_file.close()


def get_results(dir):
    with open(dir + '/robot_important_info.csv', 'rb') as robot_file:
        robot_reader = csv.reader(robot_file, delimiter=',')
        robot_updates = list(robot_reader)

        for col_index in range(len(robot_updates[0])):
            if robot_updates[0][col_index] == "task_id":
                col_taskID = col_index
            elif robot_updates[0][col_index] == "belief_state":
                col_belief_st = col_index
            elif robot_updates[0][col_index] == "real_state":
                col_real_st = col_index
            elif robot_updates[0][col_index] == "isEstimationCorrect":
                col_est = col_index
            elif robot_updates[0][col_index] == "total_disc_reward":
                col_tot_reward = col_index
            elif robot_updates[0][col_index] == "warning_received":
                col_warnings = col_index

        with open(dir + '/robot_final_results.csv', 'a') as new_file:
            wr = csv.writer(new_file, dialect='excel')
            first_row = ["task_id", "total_disc_reward", "overall_est_accur", "warnings_received"]
            wr.writerow(first_row)
            new_row = [None] * 4  # initiate a row
            task_id = 1
            tot_warnings = 0
            est_true_ct = 0
            tot_step = 0
            help_est_tot = 0 # TODO: currently the need for help estimation accuracy is calculated manually
            help_est_correct = 0
            for row_index in range(len(robot_updates)):

                if row_index == 0 or row_index == 1:  # skip the first row which has titles only
                    continue

                if int(robot_updates[row_index][col_taskID]) == task_id:
                    if int(robot_updates[row_index][col_warnings]) == 1:
                        tot_warnings += 1
                    if robot_updates[row_index][col_est] == "True":
                        est_true_ct += 1
                    tot_step += 1

                if row_index+1 == len(robot_updates) or int(robot_updates[row_index+1][col_taskID]) == task_id + 1:

                    new_row[0] = robot_updates[row_index][col_taskID]
                    new_row[1] = robot_updates[row_index][col_tot_reward]
                    new_row[2] = (float(est_true_ct) / float(tot_step)) * 100.0
                    new_row[3] = tot_warnings

                    task_id += 1
                    est_true_ct = 0
                    tot_warnings = 0
                    tot_step = 0

                    wr.writerow(new_row)  # finished a row for a task
                    new_row = [None] * 4  # initiate a row

        new_file.close()
    robot_file.close()

if __name__ == '__main__':
    work_book = open(sys.argv[1] + '/raw_data.csv', "rb")
    reader = csv.reader(work_book, delimiter=',')
    raw = list(reader)

    if sys.argv[2] == "save_robot":
        save_robot_data(sys.argv[1], raw)
    if sys.argv[2] == "robot_info":
        get_important_information(sys.argv[1])
    if sys.argv[2] == "robot_results":
        get_results(sys.argv[1])
