from morse.middleware.ros_request_manager import ros_service
from morse.core.overlay import MorseOverlay
from morse.core import status
import time

from std_srvs.srv import Trigger, TriggerResponse


class ConveyorControlOverlay(MorseOverlay):
    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        MorseOverlay.__init__(self, overlaid_object)

    #TODO: Need to make set_speed a ros_service. But currently only Trigger (boolean)type service has been found


    @ros_service(type=Trigger, name='switch_on_off')
    def switch_on_off(self):

        if self.overlaid_object.is_on:
            self.overlaid_object.set_speed(0.0)
        else:
            self.overlaid_object.set_speed(0.1) # set_speed service of conv belt automatically sets is_on variable
        return TriggerResponse(self.overlaid_object.is_on, 'Is Conveyor Belt Running?: ' + str(self.overlaid_object.is_on))