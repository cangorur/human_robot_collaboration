import csv
import sys
import numpy as np


def mov_avg(raw):
    for col_index in range(len(raw[0])):
        if raw[0][col_index] == "task_id":
            col_taskID = col_index
        elif raw[0][col_index] == "total_disc_reward":
            col_reward = col_index
        elif raw[0][col_index] == "warnings_received":
            col_warns = col_index
        elif raw[0][col_index] == "time_took":
            col_time = col_index

    with open('./avg_data.csv', 'a') as new_file:
        wr = csv.writer(new_file, dialect='excel')
        # first row:
        first_row = ["task_id", "cumulative_reward", "moving_avg_reward",
        "cumulative_warnings", "moving_avg_warnings","moving_avg_time"]
        wr.writerow(first_row)
        new_row = [None] * 6  # initiate a row

        cumulative_timeTook = 0.0
        cumulative_reward = 0.0
        cumulative_warnings = 0.0
        # cumulative_success = 0.0
        for row_index in range(len(raw)):

            if row_index == 0:  # skip the first row which has titles only
                continue
            else:
                #TODO: BURADAYIM !!!
                new_row[0] = raw[row_index][col_taskID]
                cumulative_reward = cumulative_reward + float(raw[row_index][col_reward])
                cumulative_timeTook = cumulative_timeTook + float(raw[row_index][col_time])
                cumulative_warnings = cumulative_warnings + float(raw[row_index][col_warns])
                new_row[1] = cumulative_reward
                new_row[2] = float(cumulative_reward / int(new_row[0])) # moving avg reward
                new_row[3] = cumulative_warnings
                new_row[4] = float(cumulative_warnings / int(new_row[0])) # moving avg reward
                new_row[5] = float(cumulative_timeTook / int(new_row[0])) # moving avg reward

                wr.writerow(new_row)  # finished a row for a task
                new_row = [None] * 6  # initiate a row

    new_file.close()
    print 'Moving averages are provided under avg_data.csv'

if __name__ == '__main__':
    work_book = open("./" + sys.argv[1], "rb")
    print 'Reading:' + sys.argv[1]
    reader = csv.reader(work_book, delimiter=',')
    raw = list(reader)

    mov_avg(raw)

    work_book.close()
