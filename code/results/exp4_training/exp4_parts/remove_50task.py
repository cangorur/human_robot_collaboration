import csv
import sys
import numpy as np


def remove_tasks(dir, raw):
    for col_index in range(len(raw[0])):
        if raw[0][col_index] == "task_id":
            col_id = col_index

    with open(dir + '/raw_data_v2.csv', 'a') as file:
        wr = csv.writer(file, dialect='excel')
        wr.writerow(raw[0])

        for row_index in range(len(raw)):
            if row_index == 0:
                continue
            else:
                new_id = int(raw[row_index][col_id]) - 50
                new_row = raw[row_index]
                new_row[col_id] = new_id
                wr.writerow(new_row)
    file.close()
    print 'removed'

if __name__ == '__main__':
    work_book = open(sys.argv[1] + '/raw_data.csv', "rb")
    print 'Reading:' + sys.argv[1] + '/raw_data.csv'
    reader = csv.reader(work_book, delimiter=',')
    raw = list(reader)

    remove_tasks(sys.argv[1], raw)

    work_book.close()
