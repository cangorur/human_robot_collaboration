from hrc_ros.msg import TaskState
import rospy
import pymorse
import xlwt

def callback_taskStatus(data):
    if data.who_reports == "HUMAN":


def output(filename, sheet, list1, list2, x, y, z):
    book = xlwt.Workbook()
    sh = book.add_sheet(sheet)

    variables = [x, y, z]
    x_desc = 'Display'
    y_desc = 'Dominance'
    z_desc = 'Test'
    desc = [x_desc, y_desc, z_desc]

    col1_name = 'Stimulus Time'
    col2_name = 'Reaction Time'

    # You may need to group the variables together
    # for n, (v_desc, v) in enumerate(zip(desc, variables)):
    for n, v_desc, v in enumerate(zip(desc, variables)):
        sh.write(n, 0, v_desc)
        sh.write(n, 1, v)

    n += 1

    sh.write(n, 0, col1_name)
    sh.write(n, 1, col2_name)

    for m, e1 in enumerate(list1, n + 1):
        sh.write(m, 0, e1)

    for m, e2 in enumerate(list2, n + 1):
        sh.write(m, 1, e2)

    book.save(filename)

if __name__ == "__main__":
    rospy.init_node('results_to_excel')
    rospy.Subscriber("/hrc_task_manager/task_status", TaskState, callback_taskStatus)
    rospy.loginfo("Results are being saved as excel files!")
    for i in range(30):
        filename = test_results
        sheet = human_data

        output(filename, sheet, list1, list2, i, i+10, i+20)

    rospy.spin()