from morse.middleware.ros_request_manager import ros_service
from morse.core.overlay import MorseOverlay
from morse.core import status
import threading
import time
import rospy
from std_msgs.msg import String

from std_srvs.srv import Trigger, TriggerResponse


class HumanControlAndMonitor(MorseOverlay):
    countGrasp = 0
    countReset = 10

    def debug(self,string):
        if (1==1):
            rospy.loginfo(string)

    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        MorseOverlay.__init__(self, overlaid_object)

        # if current action is executed
        self.is_reset = False  # if human has returned to the initial position
        #self.overlaid_object.is_wa = False  # has human walked away
        #self.overlaid_object.is_la = False  # has human looked around
        #self.overlaid_object.is_wr = False  # has human warned the robot
        #self.overlaid_object.is_gr = False  # has human grasped the object
        #self.overlaid_object.is_ag = False  # has human attempted to grasp the object

	#self.overlaid_object.warn_robot
	#warn_robot(self)	#does not work that way!
        
    @ros_service(type=Trigger, name='control')
    def control(self):
        # Initializes the humanController and starts the callback-loop
        rospy.Subscriber("Controller", String, self.human_control)
        rospy.spin()

    def human_control(self, data):#TODO
        # Control human
        # special actions
        if list(data.data)[0]=="1":
            # triangle-button
            self.debug("LOOKING AROUND")
            rospy.loginfo("DATA: %s", data.data)
            self.look_around()
        if list(data.data)[1]=="1":
            # circle-button
            self.debug("WALK AWAY")
            rospy.loginfo("DATA: %s", data.data)
            self.walk_away()
        if (list(data.data)[2]=="1" and self.countGrasp==0):
            # x-button
            self.debug("SUCCESSFUL GRASP")
            rospy.loginfo("DATA: %s", data.data)
            self.grasp()
            self.countGrasp = self.countReset
        if list(data.data)[3]=="1":
            # square-button
            self.debug("WARNING THE ROBOT")
            rospy.loginfo("DATA: %s", data.data)
            self.warn_robot()
####################
        if (list(data.data)[4]=="1" and list(data.data)[6]=="1"):
            # l1+l2
            self.debug("LOOKING TO THE LEFT")
            rospy.loginfo("DATA: %s", data.data)
            self.overlaid_object.look_left()
        if (list(data.data)[5]=="1" and list(data.data)[7]=="1"):
            # r1+r2
            self.debug("LOOKING TO THE RIGHT")
            rospy.loginfo("DATA: %s", data.data)
            self.overlaid_object.look_right()
        if (list(data.data)[4]=="1" and list(data.data)[5]=="1"):
            # l1+r1
            self.debug("LOOKING UP")
            rospy.loginfo("DATA: %s", data.data)
            self.overlaid_object.look_up()
        if (list(data.data)[6]=="1" and list(data.data)[7]=="1"):
            # l2+r2
            self.debug("LOOKING DOWN")
            rospy.loginfo("DATA: %s", data.data)
            self.overlaid_object.look_down()
####################
        if list(data.data)[19]=="1":
            # right analogstick - turning clockwise
            self.debug("BENDING DOWN")
            rospy.loginfo("DATA: %s", data.data)
            self.bow()

        # for DEBUG-Purposes
        if list(data.data)[9]=="1":
            #for debug-purposes
            self.debug("RESETING THE HUMAN")
            rospy.loginfo("DATA: %s", data.data)
            self.reset()
        # different counts if a sequence should only be played once
        if self.countGrasp>0:
            self.countGrasp = self.countGrasp - 1


    def look_left(self):
        self.overlaid_object.look_left()
        return TriggerResponse(True, 'human: look left')

    @ros_service(type=Trigger, name='is_ov')
    def is_ov(self):
        # return if the object is visible to the human

        if self.overlaid_object.is_ov:
            return TriggerResponse(True, 'human: is_ov True')
        else:
            return TriggerResponse(False, 'human: is_ov False')

    @ros_service(type=Trigger, name='is_oir')
    def is_oir(self):
        # return if the object is reachable to grasp for the human

        if self.overlaid_object.is_oir:
            return TriggerResponse(True, 'human: is_oir True')
        else:
            return TriggerResponse(False, 'human: is_oir False')

    @ros_service(type=Trigger, name='is_ho')
    def is_ho(self):
        # return if the human has the object

        if self.overlaid_object.is_ho:
            return TriggerResponse(True, 'human: is_ho True')
        else:
            return TriggerResponse(False, 'human: is_ho False')

    @ros_service(type=Trigger, name='is_a0')
    def is_a0(self):
        # return if the human is attempting to grasp

        if self.overlaid_object.is_ag or self.overlaid_object.is_gr:  # a0: grasp attempt
            return TriggerResponse(True, 'human: is_a0 True')
        else:
            return TriggerResponse(False, 'human: is_a0 False')

    @ros_service(type=Trigger, name='is_a2')
    def is_a2(self):
        # return if the human is staying idle, i.e. no action taken

        if (not self.overlaid_object.is_wa) and (
                not self.overlaid_object.is_wr) and (not self.overlaid_object.is_gr) and (
                not self.overlaid_object.is_ag):  # a2: idle
            return TriggerResponse(True, 'human: is_a2 True')
        else:
            return TriggerResponse(False, 'human: is_a2 False')

    @ros_service(type=Trigger, name='is_a4')
    def is_a4(self):
        # return if the human is warning the robot

        if self.overlaid_object.is_wr:  # a4: warn robot
            return TriggerResponse(True, 'human: is_a4 True')
        else:
            return TriggerResponse(False, 'human: is_a4 False')

    @ros_service(type=Trigger, name='reset')
    def reset(self):
        if not self.is_reset:
            self.is_reset = True
            self.overlaid_object.reset()
            self.is_reset = False
            return TriggerResponse(True, 'human: reset')
        else:
            return TriggerResponse(True, 'human: action(reset) is not finished')

    @ros_service(type=Trigger, name='attempt_and_cancel')
    def attempt_and_cancel(self):
        self.overlaid_object.attempt_and_cancel()
        return TriggerResponse(True, 'human: cancel actions')

    @ros_service(type=Trigger, name='obj_reset')
    def obj_reset(self):
        self.overlaid_object.obj_reset()
        self.is_reset = False
        return TriggerResponse(True, 'object: reset')

    @ros_service(type=Trigger, name='stay_idle')
    def stay_idle(self):
        self.overlaid_object.stay_idle()
        return TriggerResponse(True, 'human: stay idle')

    @ros_service(type=Trigger, name='walk_away')
    def walk_away(self):
        if not self.overlaid_object.is_wa:
            t1 = threading.Thread(target=self.overlaid_object.walk_away)
            t1.start()
            return TriggerResponse(True, 'human: walk away')
        else:
            return TriggerResponse(True, 'human: action(walk away) is not finished')

    @ros_service(type=Trigger, name='sit_down')
    def sit_down(self):
        if not self.overlaid_object.is_sd:
            t1 = threading.Thread(target=self.overlaid_object.sit_down)
            t1.start()
            return TriggerResponse(True, 'human: sitting down')
        else:
            return TriggerResponse(True, 'human: action(sitting down) is not finished')

    @ros_service(type=Trigger, name='stand_up')
    def stand_up(self):
        if self.overlaid_object.is_sd:
            t1 = threading.Thread(target=self.overlaid_object.stand_up)
            t1.start()
            return TriggerResponse(True, 'human: standing up')
        else:
            return TriggerResponse(True, 'human: action(standing up) is not finished')

    @ros_service(type=Trigger, name='look_around')
    def look_around(self):
        if not self.overlaid_object.is_la:
            t1 = threading.Thread(target=self.overlaid_object.look_around())
            t1.start()
            return TriggerResponse(True, 'human: look around')
        else:
            return TriggerResponse(True, 'human: action(look around) is not finished')

    @ros_service(type=Trigger, name='warn_robot')
    def warn_robot(self):
        if not self.overlaid_object.is_wr:
            t1 = threading.Thread(target=self.overlaid_object.warn_robot)
            t1.start()
            return TriggerResponse(True, 'human: warn robot')
        else:
            return TriggerResponse(True, 'human: action(warn robot) is not finished')

    # grasp
    @ros_service(type=Trigger, name='grasp')
    def grasp(self):
        if self.overlaid_object.is_ho:
            return TriggerResponse(True, 'human: already had object')
        else:
            # TODO: here is also to be threaded. Right now as it is a successful grasp robot waits for it to be finished
            if not self.overlaid_object.is_gr:
                # success = self.overlaid_object.grasp_animated()
                success = self.overlaid_object.grasp()
                return TriggerResponse(True, 'human: grasp object')
            else:
                return TriggerResponse(True, 'human: action(grasp object) is not finished')

    # attempt to grasp
    @ros_service(type=Trigger, name='attempt_grasp')
    def attempt_grasp(self):
        if self.overlaid_object.is_ho:
            return TriggerResponse(True, 'human: already had object')
        else:
            if not self.overlaid_object.is_ag:
                t1 = threading.Thread(target=self.overlaid_object.attempt_grasp)
                t1.start()
                return TriggerResponse(True, 'human: attempt_grasp')
            else:
                return TriggerResponse(True, 'human: action(attempt_grasp) is not finished')

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
