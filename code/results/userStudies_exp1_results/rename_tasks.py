import csv
import sys
import numpy as np


def renumber_tasks(dir, raw, new_id):
    for col_index in range(len(raw[0])):
        if raw[0][col_index] == "task_id":
            col_task_id = col_index
        if raw[0][col_index] == "subtask_id":
            col_subtask_id = col_index

    with open(dir + '/raw_renumbered.csv', 'a') as file:
        wr = csv.writer(file, dialect='excel')
        wr.writerow(raw[0])

        for row_index in range(len(raw)):
            if (row_index != 0):
                raw[row_index][col_task_id] = new_id
                #raw[row_index][col_subtask_id]

                wr.writerow(raw[row_index])

    file.close()
    print 'INFO: task ids renumbered according to the turn numbers'

if __name__ == '__main__':
    work_book = open(sys.argv[1] + '/raw_data.csv', "rb")
    print 'Reading:' + sys.argv[1] + '/raw_data.csv'
    reader = csv.reader(work_book, delimiter=',')
    raw = list(reader)

    renumber_tasks(sys.argv[1], raw, sys.argv[2])

    work_book.close()
