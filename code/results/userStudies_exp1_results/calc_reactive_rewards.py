import csv
import sys
import numpy as np
import math


def calc_rewards(dir, raw):
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
            col_secs = col_index + 1
            col_nsecs = col_index + 2
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

    with open(dir + '/robot_updates_new.csv', 'a') as file:
        wr = csv.writer(file, dialect='excel')
        first_row = raw[0]
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

            if (step_count > prev_step_count or row_index == len(raw)-1) and (sensors_informed):
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
                    init_time = int(raw[row_index][col_secs])
                    print subtask_id, init_time
                    subtask_init = False

                curr_time = int(raw[row_index][col_secs])
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
    print 'INFO: reward values are calculated and added to the reactive robot'

if __name__ == '__main__':
    work_book = open(sys.argv[1] + '/raw_data.csv', "rb")
    print 'Reading:' + sys.argv[1] + '/raw_data.csv'
    reader = csv.reader(work_book, delimiter=',')
    raw = list(reader)

    calc_rewards(sys.argv[1], raw)

    work_book.close()
