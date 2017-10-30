import csv
import sys


def get_distribution(dist):

        for col_index in range(len(dist[0])):
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
    work_book = open(sys.argv[1], "rb")
    reader = csv.reader(work_book, delimiter=',')
    dist = list(reader)

    if sys.argv[2] == "distribute":
        get_distribution(dist)
