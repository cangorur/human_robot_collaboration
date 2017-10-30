from morse.middleware.ros_request_manager import ros_service
from morse.core.overlay import MorseOverlay
import threading
from morse.core import status
import time

from std_srvs.srv import Trigger, TriggerResponse


class RobotControlAndMonitor(MorseOverlay):
    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        MorseOverlay.__init__(self, overlaid_object)

        # if is object at hand
        self.is_obj = False

        # if current action is executed
        self.is_gr = False  # has grasped
        self.is_pl = False  # has planned for grasping
        self.is_po = False  # has pointed out
        self.is_ca = False  # has canceled

    @ros_service(type=Trigger, name='cancel_action')
    def cancel_action(self):
        if not self.is_ca:
            self.is_ca = True
            self.overlaid_object.cancel_action()

            self.is_obj = False
            self.is_gr = False
            self.is_po = False
            self.is_ca = False
            self.is_pl = False

            return TriggerResponse(True, 'robot: cancel action')
        else:
            return TriggerResponse(True, 'robot: action(cancel action) is not finished')

    @ros_service(type=Trigger, name='is_ho')
    def is_ho(self):
        if self.overlaid_object.is_ho:
            return TriggerResponse(True, 'robot: is_ho True')
        else:
            return TriggerResponse(False, 'robot: is_ho False')

    @ros_service(type=Trigger, name='reset')
    def reset(self):
        self.overlaid_object.reset()
        self.is_obj = False
        self.is_gr = False
        self.is_po = False
        self.is_ca = False
        self.is_pl = False
        return TriggerResponse(True, 'robot: reset')

    # distance
    @ros_service(type=Trigger, name='distance')
    def distance(self):
        dist = self.overlaid_object.distance()
        if (dist < 1.2):
            return TriggerResponse(True, 'robot: object in range' + str(dist))
        else:
            return TriggerResponse(True, 'robot: object out of range' + str(dist))

    # grasp
    @ros_service(type=Trigger, name='grasp')
    def grasp(self):
        if self.is_obj:
            return TriggerResponse(True, 'robot: already had object')
        else:
            if not self.is_gr:
                # if the robot has not planned for the grasp yet
                if not self.is_pl:
                    self.planning_for_grasping()
                self.is_gr = True
                t1 = threading.Thread(target=self.overlaid_object.grasp)
                t1.start()
                self.is_gr = False
                self.is_pl = False  # after the grasping process there needs to be replanning for the upcoming grasp
                #if success:
                self.is_obj = True
                return TriggerResponse(True, 'robot: grasp object')
            else:
                return TriggerResponse(True, 'robot: action(grasp object) is not finished')

    # planning for grasping. Note that every other action taken breaks the plan. So the robot should redo
    @ros_service(type=Trigger, name='planning_for_grasping')
    def planning_for_grasping(self):
        if not self.is_pl:
            time.sleep(3)  ### SLEEP FOR ROBOT GRASPING ! ###
            self.is_pl = True
            return TriggerResponse(True, 'robot: planning for the grasping')
        else:
            self.is_pl = True
            return TriggerResponse(True, 'robot: planning for the grasping')

    # point to object
    @ros_service(type=Trigger, name='point_to_obj')
    def point_to_obj(self):
        if not self.is_po:
            # if the robot has not planned for the move yet
            if not self.is_pl:
                self.planning_for_grasping()
            self.is_po = True
            self.overlaid_object.pointToObj()
            self.is_po = False
            self.is_pl = False
            return TriggerResponse(True, 'robot: point to object')
        else:
            return TriggerResponse(True, 'robot: action(point to object) is not finished')
