#!/usr/bin/env python

from hrc_ros.srv import *
import rospy
import threading
import time

from hrc_ros.msg import TraySensor
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_msgs.msg import String


class RobotControlAndMonitor():
    def __init__(self):
        # Call the constructor of the parent class

        # if current action is executed
        self.is_ho = False  # has the object
        self.is_gr = False  # has grasped
        self.is_pl = False  # has planned for grasping
        self.is_po = False  # has pointed out
        self.is_ca = False  # has canceled
        self.cancel = False

        self.is_reset = False
        self.is_success = False # when the task is successful
        self.is_failure = False # when the task has failed

        self.inform_status = rospy.Publisher('~informs_status', String, queue_size=10)
        rospy.Subscriber("/production_line/tray_sensors", TraySensor, self.task_status_callback)

        # Robot action command services. Once called the human executes those actions
        ac1 = rospy.Service('~reset', Trigger, self.reset)
        ac2 = rospy.Service('~cancel_action', Trigger, self.cancel_action)
        ac3 = rospy.Service('~grasp', Trigger, self.grasp)
        ac4 = rospy.Service('~planning_for_motion', Trigger, self.planning_for_motion)
        ac5 = rospy.Service('~point_to_obj', Trigger, self.point_to_obj)

        info1 = rospy.Service('~is_ho', Trigger, self.get_ho)

    def reset(self, req):

        self.cancel = False

        self.is_ho = False
        self.is_gr = False
        self.is_po = False
        self.is_ca = False
        self.is_pl = False
        self.is_reset = False
        self.is_success = False # when the task is successful
        rospy.loginfo("Robot agent: reset is successful!")
        return TriggerResponse(True, 'robot: reset')

    def cancel_action(self, req):

        if not self.is_ca:
            self.is_ca = True
            t1 = threading.Thread(target=self.cancel_act)
            t1.start()
            self.is_gr = False
            self.is_po = False
            self.is_ca = False
            self.is_pl = False
            self.is_ho = False

            return TriggerResponse(True, 'robot: cancel action')
        else:
            return TriggerResponse(True, 'robot: action(cancel action) is not finished')

    def get_ho(self, req):
        if self.is_ho:
            return TriggerResponse(True, 'robot: is_ho True')
        else:
            return TriggerResponse(False, 'robot: is_ho False')

    def grasp(self, req):
        if self.is_ho:
            return TriggerResponse(True, 'robot: already had object')
        else:
            if not self.is_gr:
                # if the robot has not planned for the grasp yet
                if not self.is_pl:
                    self.planning_for_motion_act()
                self.is_gr = True
                t1 = threading.Thread(target=self.grasp_act)
                t1.start()
                self.is_gr = False
                self.is_pl = False  # after the grasping process there needs to be replanning for the upcoming grasp
                #if success:
                self.is_obj = True
                return TriggerResponse(True, 'robot: graspped object')
            else:
                return TriggerResponse(True, 'robot: action(grasp object) is not finished')

    def planning_for_motion(self, req):
        if not self.is_pl:
            t1 = threading.Thread(target=self.planning_for_motion_act)
            t1.start()
            return TriggerResponse(True, 'robot: planning for the motion')
        else:
            self.is_pl = True
            return TriggerResponse(True, 'robot: planning for the motion')

    def point_to_obj(self, req):
        if not self.is_po:
            # if the robot has not planned for the move yet
            if not self.is_pl:
                self.planning_for_motion_act()
            self.is_po = True
            t1 = threading.Thread(target=self.pointToObj_act)
            t1.start()
            self.is_po = False
            self.is_pl = False
            return TriggerResponse(True, 'robot: point to object')
        else:
            return TriggerResponse(True, 'robot: action(point to object) is not finished')


    #### FUNCTIONS BELOW ARE THE ACTUAL ACTION EXECUTION ####
    def cancel_act(self):

        self.cancel = True
        #self.obj_reset()
        time.sleep(1)
        self.is_obj = False
        if self.is_ho:
            self.is_ho = False
            self.inform_status.publish("released_object") # informing the simulated human about the state
        self.is_gr = False
        self.is_po = False
        self.is_ca = False
        self.is_pl = False
        rospy.loginfo("Robot agent: canceled the actions!")


    def grasp_act(self):

        # approach to the object with both arms

        N = 20
        for i in range(N):

            if self.cancel:
                self.cancel_act()
                return False

            time.sleep(0.06)

        if (not self.is_failure and not self.is_success):

            # close the shoulders to hold the object
            N = 20
            for i in range(N):
                if self.cancel:
                    self.cancel_act()
                    return True
                time.sleep(0.1)
            # self.finger_open()

            N = 20
            # lift the object up
            if (not self.is_failure):
                self.is_ho = True
                self.inform_status.publish("got_object") # informing the simulated human about the state
            else:
                self.cancel_act()
                self.is_ho = False
                return

            for i in range(N):
                if self.cancel:
                    self.cancel_act()
                    return True
                time.sleep(0.1)

            time.sleep(0.1)
            # Turning towards the processed storage tray
            for i in range(N):
                # When turning the robot cannot see human warning
                if self.cancel:
                    self.cancel_act()
                    return True
                time.sleep(0.1)

            # dropping the object off
            for i in range(N):
                # When turned the robot cannot see the human warning
                if self.cancel:
                    self.cancel_act()
                    return True
                time.sleep(0.1)
            N = 20
            for i in range(N):
                time.sleep(0.1)
            time.sleep(0.5)
            # obj.worldPosition = [7.7 - 0.6, -3.04, 0.80]

            # Turning back to the conveyor belt
            N = 20
            for i in range(N):
                if self.cancel:
                    return True
                time.sleep(0.1)

            #self.finger_close()
            self.is_ho = False
            self.inform_status.publish("released_object") # informing the simulated human about the state

            time.sleep(0.5)
            self.is_success = True
            self.inform_status.publish("success") # informing the simulated human about the state
            rospy.loginfo("Robot agent: grasp was successful. Task is finished!")

            return True
        else:
            self.is_ho = False
            rospy.loginfo("Robot agent: grasp is failed!")
            return False

    def planning_for_motion_act(self):

        time.sleep(4)
        self.is_pl = True
        rospy.loginfo("Robot agent: planned for the motion!")

    def pointToObj_act(self):

        if (not self.is_failure and not self.is_success):
            N = 5

            for i in range(N):
                if self.cancel:
                    self.cancel_act()
                    return False
                time.sleep(0.1)

            time.sleep(0.5)
            for i in range(N):
                if self.cancel:
                    self.cancel_act()
                    return True
            time.sleep(0.5)
            for i in range(N):
                if self.cancel:
                    self.cancel_act()
                    return False
                time.sleep(0.1)

            time.sleep(0.5)
            for i in range(N):
                if self.cancel:
                    self.cancel_act()
                    return True
            time.sleep(0.5)
            rospy.loginfo("Robot agent: pointed out the object!")

    def task_status_callback(self, data):

        if data.tray_id == 'tray_unprocessed':
            self.is_failure = data.occupied
            if self.is_failure:
                rospy.loginfo("Robot agent: task is failed!")
        elif data.tray_id == 'tray_human':
            self.is_success = data.occupied
            if self.is_success:
                rospy.loginfo("Robot agent: human has accomplished the task!")


if __name__ == "__main__":
    rospy.init_node('robot')
    human = RobotControlAndMonitor()
    rospy.loginfo("Robot agent simulator is ready!")
    rospy.spin()
