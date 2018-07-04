#!/usr/bin/env python

from hrc_ros.srv import *
import rospy
import threading
import time

from std_srvs.srv import Trigger, TriggerResponse


class HumanControlAndMonitor():
    def __init__(self):
        # Call the constructor of the parent class

        # these are to feed the human action back to the system (current state)
        self.is_reset = False
        self.is_wa = False  # is human walking away
        self.is_gr = False  # is human grasping
        self.is_sd = False  # is human grasping

        self.is_la = False  # is human looking around
        self.is_wr = False  # is human warning the robot
        self.is_ag = False  # is human attempting to grasp

        self.is_ov = True  # is the object visible to human
        self.is_oir = True  # is the object reachable to human
        self.is_ho = False  # if human has the object

        # Observation response services. Let the requester know about the current human action and situation
        os1 = rospy.Service('~is_ov', Trigger, self.get_ov)
        os2 = rospy.Service('~is_oir', Trigger, self.get_oir)
        os3 = rospy.Service('~is_ho', Trigger, self.get_ho)
        os4 = rospy.Service('~is_a0', Trigger, self.get_a0)
        os5 = rospy.Service('~is_a2', Trigger, self.get_a2)
        os6 = rospy.Service('~is_a4', Trigger, self.get_a4)

        # Human action command services. Once called the human executes those actions
        ac1 = rospy.Service('~reset', Trigger, self.reset)
        ac2 = rospy.Service('~attempt_and_cancel', Trigger, self.attempt_and_cancel)
        ac3 = rospy.Service('~stay_idle', Trigger, self.stay_idle)
        ac4 = rospy.Service('~walk_away', Trigger, self.walk_away)
        ac5 = rospy.Service('~sit_down', Trigger, self.sit_down)
        ac6 = rospy.Service('~stand_up', Trigger, self.stand_up)
        ac7 = rospy.Service('~look_around', Trigger, self.look_around)
        ac8 = rospy.Service('~warn_robot', Trigger, self.warn_robot)
        ac9 = rospy.Service('~grasp', Trigger, self.grasp)
        ac10 = rospy.Service('~attempt_grasp', Trigger, self.attempt_grasp)

    def get_ov(self, req):
        # return if the object is visible to the human

        if self.is_ov:
            return TriggerResponse(True, 'human: is_ov True')
        else:
            return TriggerResponse(False, 'human: is_ov False')

    def get_oir(self, req):
        # return if the object is reachable to grasp for the human

        if self.is_oir:
            return TriggerResponse(True, 'human: is_oir True')
        else:
            return TriggerResponse(False, 'human: is_oir False')

    def get_ho(self, req):
        # return if the human has the object

        if self.is_ho:
            return TriggerResponse(True, 'human: is_ho True')
        else:
            return TriggerResponse(False, 'human: is_ho False')

    def get_a0(self, req):
        # return if the human is attempting to grasp

        if self.is_ag or self.is_gr:  # a0: grasp attempt
            return TriggerResponse(True, 'human: is_a0 True')
        else:
            return TriggerResponse(False, 'human: is_a0 False')

    def get_a2(self, req):
        # return if the human is staying idle, i.e. no action taken

        if (not self.is_wa) and (not self.is_wr) and (not self.is_gr) and (not self.is_ag):  # a2: idle
            return TriggerResponse(True, 'human: is_a2 True')
        else:
            return TriggerResponse(False, 'human: is_a2 False')

    def get_a4(self, req):
        # return if the human is warning the robot

        if self.is_wr:  # a4: warn robot
            return TriggerResponse(True, 'human: is_a4 True')
        else:
            return TriggerResponse(False, 'human: is_a4 False')

    def reset(self, req):

        if not self.is_reset:

            self.is_reset = True

            if self.is_la:
                self.look_back()

            if self.is_ag or self.is_gr:
                self.attempt_grasp_back()

            if self.is_wr:
                self.warn_robot_back()

            if self.is_wa:
                self.walk_back()

            if self.is_sd:
                self.stand_up()  # TODO: put stand_up action here

            self.is_wa = False
            self.is_gr = False

            self.is_la = False
            self.is_wr = False
            self.is_ag = False
            self.is_sd = False

            self.is_ov = True
            self.is_oir = True
            self.is_ho = False
            self.is_reset = False
            rospy.loginfo("Human agent: reset is completed!")
            return TriggerResponse(True, 'human: reset')
        else:
            return TriggerResponse(True, 'human: action(reset) is not finished')

    def attempt_and_cancel(self, req):
        self.attempt_and_cancel_act()
        return TriggerResponse(True, 'human: cancel actions')

    def stay_idle(self, req):
        self.stay_idle_act()
        return TriggerResponse(True, 'human: stay idle')

    def walk_away(self, req):
        if not self.is_wa:
            t1 = threading.Thread(target=self.walk_away_act)
            t1.start()
            return TriggerResponse(True, 'human: walk away')
        else:
            return TriggerResponse(True, 'human: action(walk away) is not finished')

    def sit_down(self, req):
        if not self.is_sd:
            t1 = threading.Thread(target=self.sit_down_act)
            t1.start()
            return TriggerResponse(True, 'human: sitting down')
        else:
            return TriggerResponse(True, 'human: action(sitting down) is not finished')

    def stand_up(self, req):
        if self.is_sd:
            t1 = threading.Thread(target=self.stand_up_act)
            t1.start()
            return TriggerResponse(True, 'human: standing up')
        else:
            return TriggerResponse(True, 'human: action(standing up) is not finished')

    def look_around(self, req):
        if not self.is_la:
            t1 = threading.Thread(target=self.look_around_act())
            t1.start()
            return TriggerResponse(True, 'human: look around')
        else:
            return TriggerResponse(True, 'human: action(look around) is not finished')

    def warn_robot(self, req):
        if not self.is_wr:
            t1 = threading.Thread(target=self.warn_robot_act)
            t1.start()
            return TriggerResponse(True, 'human: warn robot')
        else:
            return TriggerResponse(True, 'human: action(warn robot) is not finished')

    # grasp
    # TODO: this grasp action can be threaded. Also I can include a cancel action to allow other actions to be called for the human. This cancel can be inspired from the robot cancel
    def grasp(self, req):
        if self.is_ho:
            return TriggerResponse(True, 'human: already had object')
        else:
            # TODO: here is also to be threaded. Right now as it is a successful grasp robot waits for it to be finished
            if not self.is_gr:
                t1 = threading.Thread(target=self.grasp_act)
                t1.start()
                return TriggerResponse(True, 'human: grasp object')
            else:
                return TriggerResponse(True, 'human: action(grasp object) is not finished')

    # attempt to grasp
    def attempt_grasp(self, req):
        if self.is_ho:
            return TriggerResponse(True, 'human: already had object')
        else:
            if not self.is_ag:
                t1 = threading.Thread(target=self.attempt_grasp_act)
                t1.start()
                return TriggerResponse(True, 'human: attempt_grasp')
            else:
                return TriggerResponse(True, 'human: action(attempt_grasp) is not finished')

    #### FUNCTIONS BELOW ARE THE ACTUAL ACTION EXECUTION ####
    def attempt_and_cancel_act(self):

        if self.is_gr or self.is_ag:
            self.attempt_grasp_back()
        if self.is_la:
            self.look_back()
        if self.is_wr:
            self.warn_robot_back()

        self.is_ag = True
        self.is_gr = True
        self.is_ho = False

        N = 50
        for i in range(N):
            time.sleep(0.01)
        time.sleep(1)

        self.attempt_grasp_back()
        rospy.loginfo("Human agent: attempted to grasp but canceled right now!")

    def stay_idle_act(self):

        if self.is_la:
            self.look_back()

        if self.is_ag or self.is_gr:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        self.is_ov = True
        # self.is_oir = True
        rospy.loginfo("Human agent: idling now!")

    def walk_away_act(self):

        if self.is_wa:
            return
        else:
            self.is_wa = True

        if self.is_la:
            self.look_back()

        if self.is_ag or self.is_gr:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        self.is_ov = False
        self.is_oir = False

        time.sleep(5)
        rospy.loginfo("Human agent: walk away completed!")

    def sit_down_act(self):

        if self.is_sd:
            return
        else:
            self.is_sd = True

        if self.is_la:
            self.look_back()

        if self.is_ag or self.is_gr:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        time.sleep(1.5)

        self.is_ov = True
        self.is_oir = True
        rospy.loginfo("Human agent: sitting down completed!")

    def stand_up_act(self):

        if not self.is_sd:
            return
        else:
            self.is_sd = False

        if self.is_la:
            self.look_back()

        if self.is_ag or self.is_gr:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        time.sleep(1.5)
        rospy.loginfo("Human agent: stand up completed!")

    def look_around_act(self):

        if self.is_la:
            return

        if self.is_wa:
            self.walk_back()

        if self.is_ag or self.is_gr:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        time.sleep(2)

        self.is_la = True
        self.is_ov = False
        rospy.loginfo("Human agent: look around completed!")

    def warn_robot_act(self):

        if self.is_wr:
            return
        else:
            self.is_wr = True

        if self.is_wa:
            self.walk_back()

        if self.is_la:
            self.look_back()

        if self.is_ag or self.is_gr:
            self.attempt_grasp_back()

        time.sleep(1.5)
        self.is_wr = True
        rospy.loginfo("Human agent: warn the robot completed!")

    def grasp_act(self):

        if self.is_wa:
            self.walk_back()
        if self.is_gr or self.is_ag:
            self.attempt_grasp_back()
        if self.is_la:
            self.look_back()
        if self.is_wr:
            self.warn_robot_back()
        self.is_gr = True

        # bend to grab the object
        time.sleep(1.5)
        self.is_gr = True

        # straighten up a bit to lift the object
        time.sleep(0.5)
        self.is_ho = True

        # turn right 90 degree
        time.sleep(0.5)

        # bend a bit down to drop the object
        time.sleep(0.5)

        # standing direct after releasing the object. Putting the human back its initial pose and gesture
        time.sleep(0.5)
        self.is_ho = False

        # turn left 90 degree
        time.sleep(0.25)
        rospy.loginfo("Human agent: grasp completed!")

    def attempt_grasp_act(self):

        if self.is_wa:
            self.walk_back()
        if self.is_gr or self.is_ag:
            self.attempt_grasp_back()
        if self.is_la:
            self.look_back()
        if self.is_wr:
            self.warn_robot_back()

        self.is_ag = True
        self.is_gr = True
        self.is_ho = False

        # bend to grab the object
        time.sleep(1.5)

        # straighten up a bit to lift the object
        time.sleep(0.5)

        # push hard to lift but no success (but moves up and down)
        time.sleep(0.5)

        # gave up, heads down first then miserably pushes the product
        time.sleep(2.5)

        self.attempt_grasp_back()
        self.is_ho = False
        rospy.loginfo("Human agent: attempt to grasp completed!")

    # functions below are to cancel, or revers the actions selected above.
    def look_back(self):

        time.sleep(0.5)

        self.is_la = False
        self.is_ov = True

    def attempt_grasp_back(self):

        time.sleep(0.75)

        self.is_ag = False
        self.is_gr = False


    def warn_robot_back(self):

        time.sleep(0.2)
        self.is_wr = False

    def walk_back(self):

        if self.is_wa:
            self.is_wa = False
        else:
            return

        time.sleep(5)

        self.is_ov = True
        self.is_oir = True


    # manipulation mode
    '''
    @ros_service(type = Trigger, name = 'toggle_manipulation')
    def toggle_manipulation(self):
        is_manipulation = self.overlaid_object.toggle_manipulation()
        if is_manipulation:
            return TriggerResponse(True, 'manipulation mode is now on!')
        else:
            return TriggerResponse(True, 'manipulation mode is now off!')

    @ros_service(type = Trigger, name = 'get_manipulation_state')
    def get_manipulation_state(self):
        is_manipulation = self.overlaid_object.manipulation_state()
        if is_manipulation:
            return TriggerResponse(True, 'on')
        else:
            return TriggerResponse(True, 'off')
    '''
if __name__ == "__main__":
    rospy.init_node('human')
    human = HumanControlAndMonitor()
    rospy.loginfo("Human agent simulator is ready!")
    rospy.spin()