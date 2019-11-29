import csv
import sys
import math


def save_robot_data(dir, raw):
    for col_index in range(len(raw[0])):
        if raw[0][col_index] == "task_id":
            col_task_id = col_index
        if raw[0][col_index] == "subtask_id":
            col_subtask_id = col_index
        if raw[0][col_index] == "step_count":
            col_stepCount = col_index
        if raw[0][col_index] == "who_reports":
            col_reporter = col_index
        if raw[0][col_index] == "update_received_time":
            col_updSecs = col_index + 1
            col_updNsecs = col_index + 2
        if raw[0][col_index] == "action_taken_time":
            col_actSecs = col_index + 1
            col_actNsecs = col_index + 2
        if raw[0][col_index] == "mapped_observation_raw":
            col_observation = col_index
        if raw[0][col_index] == "subtask_status":
            col_subStatus = col_index
        if raw[0][col_index] == "subtask_duration":
            col_subDuration = col_index
        if raw[0][col_index] == "immediate_reward":
            col_immReward = col_index
        if raw[0][col_index] == "total_disc_reward":
            col_totReward = col_index

    with open(dir + '/robot_update.csv', 'a') as file:
        wr = csv.writer(file, dialect='excel')
        first_row = raw[0]
        first_row[col_updSecs] = "update_secs"
        first_row[col_updNsecs] = "update_nsecs"
        first_row[col_actSecs] = "acted_secs"
        first_row[col_actNsecs] = "acted_nsecs"
        wr.writerow(first_row)
        total_disc_reward = 0.0
        immediate_reward = 0.0
        immediate_reward_robot = 0.0
        discount_factor = 0.98
        task_id = 0
        subtask_id = -1
        subtask_init = True
        init_time = 0
        sensors_informed = False
        prev_step_count = 0
        robot_should_inform = False
        prev_robot_row = [None] * len(raw[0])

        for row_index in range(len(raw)):

            if row_index == 0:
                continue

            step_count = int(raw[row_index][col_stepCount])

            if (step_count > prev_step_count or (step_count != prev_step_count and step_count == 0) or row_index == len(raw)-1) and (sensors_informed):
                # the robot hasnt informed on the prev step, so we should let the robot inform first for previous
                robot_should_inform = True

            if raw[row_index][col_reporter] == '"ROBOT"' or robot_should_inform:

                new_row = raw[row_index]

                if robot_should_inform:
                    new_row = prev_robot_row
                    new_row[col_stepCount] = prev_step_count
                    new_row[col_immReward] = immediate_reward
                    new_row[col_totReward] = total_disc_reward

                elif (int(raw[row_index][col_stepCount]) == step_count) and (int(raw[row_index][col_task_id]) == task_id):
                    immediate_reward_robot = raw[row_index][col_immReward]
                    last_robot_step = int(raw[row_index][col_stepCount])
                    new_row[col_immReward] = immediate_reward
                    new_row[col_totReward] = total_disc_reward

                sensors_informed = False # robot informed after the sensors, so lower the flag
                robot_should_inform = False
                prev_robot_row = new_row
                wr.writerow(new_row)

            task_id_new = int(raw[row_index][col_task_id])
            if task_id_new != task_id:
                total_disc_reward = 0.0
                task_id = task_id_new

            if raw[row_index][col_reporter] == '"SENSOR"':
                subtask_id_new = int(raw[row_index][col_subtask_id])
                if subtask_id_new != subtask_id:
                    #TODO: write to the robot here
                    subtask_init = True
                    subtask_id = subtask_id_new
                if raw[row_index][col_subStatus] == '"success"' or \
                        raw[row_index][col_subStatus] == '"ROBOT SUCCEEDED"':
                    immediate_reward = 6
                elif raw[row_index][col_subStatus] == '"fail"':
                    immediate_reward = -6
                else:
                    print "Sth wrong with immediate reward!"
                    immediate_reward = 0

                subtask_tick = math.floor(float(raw[row_index][col_subDuration]))

                total_disc_reward = total_disc_reward + (discount_factor**subtask_tick)*immediate_reward
                step_count = int(raw[row_index][col_stepCount])
                if (last_robot_step == step_count) and (immediate_reward_robot != immediate_reward):
                    sensors_informed = True
                    robot_should_inform = True

            if raw[row_index][col_reporter] == '"OBSERVATION"':

                if subtask_init:
                    init_time = int(raw[row_index][col_updSecs])
                    subtask_init = False

                curr_time = int(raw[row_index][col_updSecs])
                subtask_tick = curr_time - init_time

                observation = int(raw[row_index][col_observation])
                if observation == 16 or observation == 17 or observation == 18 or observation == 19 or observation == 27:
                    immediate_reward = -3.0
                else:
                    immediate_reward = 0.0

                total_disc_reward = total_disc_reward + (discount_factor**subtask_tick)*immediate_reward
                sensors_informed = True

            prev_step_count = step_count # to keep the last step informed

    file.close()

    print 'INFO: robot information is saved separately under the folder provided'

def get_important_information(dir):
    robot_file = open(dir + '/robot_update.csv', 'rb')
    robot_reader = csv.reader(robot_file, delimiter=',')
    robot_updates = list(robot_reader)

    status_file = open(dir + '/task_status.csv', 'rb')
    task_reader = csv.reader(status_file, delimiter=',')
    task_updates = list(task_reader)

    #human_file = open(dir + '/human_update.csv', 'rb')
    #human_reader = csv.reader(human_file, delimiter=',')
    #human_updates = list(human_reader)

    for col_index in range(len(task_updates[0])):
        if task_updates[0][col_index] == "task_id":
            col_taskInit_taskID = col_index
        elif task_updates[0][col_index] == "init_secs":
            col_taskInit_secs = col_index
        elif task_updates[0][col_index] == "init_nsecs":
            col_taskInit_nsecs = col_index
    '''
    for col_index in range(len(human_updates[0])):
        if human_updates[0][col_index] == "task_id":
            col_human_taskID = col_index
        elif human_updates[0][col_index] == "step_count":
            col_humanStep = col_index
        elif human_updates[0][col_index] == "human_model":
            col_humanModel = col_index
        elif human_updates[0][col_index] == "belief_state":
            col_humanBelief = col_index
    '''
    for col_index in range(len(robot_updates[0])):
        if robot_updates[0][col_index] == "task_id":
            col_taskID = col_index
        elif robot_updates[0][col_index] == "step_count":
            col_step = col_index
        elif robot_updates[0][col_index] == "acted_secs":
            col_act_secs = col_index
        elif robot_updates[0][col_index] == "acted_nsecs":
            col_act_nsecs = col_index
        elif robot_updates[0][col_index] == "robot_model":
            col_robot_model = col_index
        elif robot_updates[0][col_index] == "taken_action":
            col_action = col_index
        elif robot_updates[0][col_index] == "belief_state":
            col_belief_st = col_index
        elif robot_updates[0][col_index] == "real_state":
            col_real_st = col_index
        #elif robot_updates[0][col_index] == "isEstimationCorrect":
        #    col_est = col_index
        elif robot_updates[0][col_index] == "immediate_reward":
            col_reward = col_index
        elif robot_updates[0][col_index] == "total_disc_reward":
            col_tot_reward = col_index

    with open(dir + '/robot_important_info.csv', 'a') as new_file:
        wr = csv.writer(new_file, dialect='excel')
        first_row = ["task_id", "task_init_secs", "task_init_nsecs", "acted_secs", "acted_nsecs", "taken_action",
                     "belief_state", "real_state", "robot_model", "immediate_reward", "total_disc_reward"]
        wr.writerow(first_row)
        new_row = [None] * 11  # initiate a row
        task_id = int(robot_updates[1][col_taskID])
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
            #new_row[8] = robot_updates[row_index][col_est]
            new_row[8] = robot_updates[row_index][col_robot_model]
            new_row[9] = robot_updates[row_index][col_reward]
            new_row[10] = robot_updates[row_index][col_tot_reward]
            reward_str = robot_updates[row_index][col_reward]

            '''
            for row_human in range(len(human_updates)):
                if row_human == 0:  # skip the first row which has titles only
                    continue
                if int(human_updates[row_human][col_human_taskID]) == task_id and \
                                int(human_updates[row_human][col_humanStep]) == step_count:
                    new_row[8] = human_updates[row_human][col_humanBelief]
                    new_row[9] = human_updates[row_human][col_humanModel]
                    break
                else:
                    new_row[8] = ""
                    new_row[9] = ""
            '''
            wr.writerow(new_row)  # finished a row for a task
            new_row = [None] * 11  # initiate a row

    new_file.close()
    #human_file.close()
    status_file.close()
    robot_file.close()
    print 'INFO: robot information has been rid out of unnecessary information'

def get_results(dir):
    with open(dir + '/robot_important_info.csv', 'rb') as robot_file:
        robot_reader = csv.reader(robot_file, delimiter=',')
        robot_updates = list(robot_reader)
    with open(dir + '/sensor_update.csv', 'rb') as sensor_file:
        sensor_reader = csv.reader(sensor_file, delimiter=',')
        sensor_updates = list(sensor_reader)

        for col_index in range(len(robot_updates[0])):
            if robot_updates[0][col_index] == "task_id":
                col_taskID = col_index
            elif robot_updates[0][col_index] == "belief_state":
                col_belief_st = col_index
            elif robot_updates[0][col_index] == "real_state":
                col_real_st = col_index
            #elif robot_updates[0][col_index] == "isEstimationCorrect":
            #    col_est = col_index
            elif robot_updates[0][col_index] == "robot_model":
                col_model = col_index
            elif robot_updates[0][col_index] == "total_disc_reward":
                col_tot_reward = col_index
        for col_index in range(len(sensor_updates[0])):
            if sensor_updates[0][col_index] == "task_id":
                col_taskID_sensor = col_index
            elif sensor_updates[0][col_index] == "who_reports":
                col_whoReports = col_index
            elif sensor_updates[0][col_index] == "warnings_count_task":
                col_warnings = col_index
            elif sensor_updates[0][col_index] == "task_duration":
                col_duration = col_index
            elif sensor_updates[0][col_index] == "percentage_successful_subtasks":
                col_success = col_index
            elif sensor_updates[0][col_index] == "subtask_status":
                col_subtaskState = col_index
            elif sensor_updates[0][col_index] == "who_succeeded_subtask":
                col_whoSucceeded = col_index

        with open(dir + '/robot_final_results.csv', 'a') as new_file:
            wr = csv.writer(new_file, dialect='excel')
            first_row = ["robot_model_ID", "task_id", "warnings_received", "cumulative_warnings", "moving_avg_warnings",
                "task_duration", "cumulative_time", "moving_avg_time", "total_disc_reward", "cumulative_reward", "moving_avg_reward",
                        "success_rate", "cumulative_success", "moving_avg_success", "humans_contribution", "human_success_rate"]
            wr.writerow(first_row)
            new_row = [None] * 16  # initiate a row
            task_id = int(robot_updates[1][col_taskID])
            task_count = 1
            tot_warnings = 0
            task_duration = 0.0
            success_rate = 0.0
            human_success_ctr = 0
            human_failure_ctr = 0
            robot_success_ctr = 0
            robot_failure_ctr = 0
            est_true_ct = 0
            cumulated_reward = 0.0
            cumulated_warnings = 0.0
            cumulated_duration = 0.0
            cumulated_successRate = 0.0
            model_ = robot_updates[1][col_model]
            model_ = model_.strip('\"')
            prev_model_id, bullshit = model_.split(".")

            for row_index in range(len(robot_updates)):

                if row_index == 0:  # skip the first row which has titles only
                    continue

                task_id = int(robot_updates[row_index][col_taskID])
                if row_index+1 == len(robot_updates) or int(robot_updates[row_index+1][col_taskID]) != task_id:
                    human_success_ctr = 0
                    human_failure_ctr = 0
                    robot_success_ctr = 0
                    robot_failure_ctr = 0
                    for row_ in range(len(sensor_updates)):
                        if sensor_updates[row_][col_whoReports] == '"SENSOR"' and int(sensor_updates[row_][col_taskID_sensor]) == task_id:
                            if sensor_updates[row_][col_subtaskState] == '"success"' and \
                                sensor_updates[row_][col_whoSucceeded] == '"HUMAN"':
                                human_success_ctr += 1
                            elif sensor_updates[row_][col_subtaskState] == '"fail"' and \
                                sensor_updates[row_][col_whoSucceeded] == '"HUMAN"':
                                human_failure_ctr += 1
                            elif (sensor_updates[row_][col_subtaskState] == '"success"' or sensor_updates[row_][col_subtaskState] == '"ROBOT SUCCEEDED"') and \
                                sensor_updates[row_][col_whoSucceeded] == '"ROBOT"':
                                robot_success_ctr += 1
                            elif sensor_updates[row_][col_subtaskState] == '"fail"' and \
                                sensor_updates[row_][col_whoSucceeded] == '"ROBOT"':
                                robot_failure_ctr += 1
                            else:
                                print "Missing information for task_id:", task_id
                        if sensor_updates[row_][col_whoReports] == '"MANAGER-TASK-DONE"' and int(sensor_updates[row_][col_taskID_sensor]) == task_id:
                            tot_warnings = int(sensor_updates[row_][col_warnings])
                            task_duration = float(sensor_updates[row_][col_duration])
                            success_rate = float(sensor_updates[row_][col_success])

                    _model = robot_updates[row_index][col_model]
                    model = _model.strip('\"')
                    model_id, bullshit = model.split(".")
                    new_row[0] = model_id

                    '''
                    # Here is only for Experiment-1 where we compare different models
                    if prev_model_id != model_id or task_id == 2 or task_id == 6: # TODO: change here for ABPS experiments
                        # each model statistics are separate from each other, and task id 1 and 5 are human interacting alone
                        task_count = 1
                        cumulated_reward = 0.0
                        cumulated_warnings = 0.0
                        cumulated_duration = 0.0
                        cumulated_successRate = 0.0
                        prev_model_id = model_id
                    '''
                    new_row[1] = task_id

                    new_row[2] = tot_warnings
                    cumulated_warnings = cumulated_warnings + tot_warnings
                    new_row[3] = cumulated_warnings # cumulative warnings
                    new_row[4] = float(cumulated_warnings / task_count) # moving avg

                    new_row[5] = task_duration
                    cumulated_duration = cumulated_duration + task_duration
                    new_row[6] = cumulated_duration # cumulative task duration
                    new_row[7] = float(cumulated_duration / task_count) # moving avg

                    _reward = robot_updates[row_index][col_tot_reward]
                    reward = _reward.strip('\"')
                    new_row[8] = float(reward)
                    cumulated_reward = cumulated_reward + float(reward)
                    new_row[9] = cumulated_reward # cumulative reward
                    new_row[10] = float(cumulated_reward / task_count) # moving avg

                    new_row[11] = success_rate
                    cumulated_successRate = cumulated_successRate + success_rate
                    new_row[12] = cumulated_successRate # cumulative success rate
                    new_row[13] = float(cumulated_successRate / task_count) # moving avg

                    new_row[14] = float(human_success_ctr / (success_rate / 10)) * 100.0
                    new_row[15] = float(human_success_ctr / float(human_success_ctr + human_failure_ctr)) * 100.0

                    task_count += 1

                    tot_warnings = 0

                    wr.writerow(new_row)  # finished a row for a task
                    new_row = [None] * 16  # initiate a row

        new_file.close()
    robot_file.close()
    print 'INFO: robots final results has been saved'

if __name__ == '__main__':
    work_book = open(sys.argv[1] + '/raw_data.csv', "rb")
    reader = csv.reader(work_book, delimiter=',')
    raw = list(reader)

    if sys.argv[2] == "save_all":
        save_robot_data(sys.argv[1], raw)
        get_important_information(sys.argv[1])
        get_results(sys.argv[1])
    if sys.argv[2] == "save_robot":
        save_robot_data(sys.argv[1], raw)
    if sys.argv[2] == "robot_info":
        get_important_information(sys.argv[1])
    if sys.argv[2] == "robot_results":
        get_results(sys.argv[1])

    work_book.close()
