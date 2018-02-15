import logging

logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.robots.grasping_robot import GraspingRobot
from morse.core.services import service

import time
import math


PI = math.pi


class Human(GraspingRobot):
    """ Class definition for the human as a robot entity.

    Sub class of GraspingRobot.
    """

    def __init__(self, obj, parent=None):
        """ Call the constructor of the parent class """
        logger.info('%s initialization' % obj.name)
        GraspingRobot.__init__(self, obj, parent)

        # We define here the name of the human grasping hand:
        self.hand_name = 'Hand_Grab.R'

        self.MIN_DIST = 0.8

        # these are to feed the human action back to the system (current state)
        self.is_wa = False  # is human walking away
        self.is_gr = False  # is human grasping
        self.is_sd = False  # is human grasping

        self.is_la = False  # is human looking around
        self.is_wr = False  # is human warning the robot
        self.is_ag = False  # is human attempting to grasp

        self.is_ov = True  # is the object visible to human
        self.is_oir = True  # is the object reachable to human
        self.is_ho = False  # if human has the object

        # Default head configuration is to look down
        scene = blenderapi.scene()
        head = scene.objects['Look_Empty']
        f_speed_head = [0, 0, -0.1]
        N = 10
        for i in range(N):
            head.applyMovement([f_speed_head[0] / N, f_speed_head[1] / N, f_speed_head[2] / N], True)

        logger.info('Component initialized')

    @service
    def reset(self):
        ''' ungrasp object '''

        if self.is_la:
            self.look_back()

        if self.is_ag or self.is_gr:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        if self.is_wa:
            self.walk_back()

        if self.is_sd:
            self.stand_up() # TODO: put stand_up action here

        self.is_wa = False
        self.is_gr = False

        self.is_la = False
        self.is_wr = False
        self.is_ag = False
        self.is_sd = False

        self.is_ov = True
        self.is_oir = True
        self.is_ho = False

        human = self.bge_object
        human.worldPosition = [7.7, -1.25, 0]
        human.worldOrientation = [0, 0, -1.57]

    @service
    def obj_reset(self):
        ''' ungrasp object '''
        scene = blenderapi.scene()
        obj = scene.objects['package1']
        obj.worldPosition = [7.7, -2.1, 0.80]

    @service
    def stay_idle(self):
        """ Move human to left. """

        if self.is_la:
            self.look_back()

        if self.is_ag or self.is_gr:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        self.is_ov = True
        #self.is_oir = True

    @service
    def sit_down(self):
        """ sit human down. """

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

        self.is_ov = True
        self.is_oir = True

        armature = blenderapi.scene().objects['human.armature']

        # human.applyMovement([sit_speed, 0, 0], True)
        armature['sitDown'] = True
        armature.update()
        
        time.sleep(.5)
        """ Stops animating the human sitting down / standing up. """
        armature['sitDown'] = False
        armature.update()

    @service
    def stand_up(self):
        """ sit human down. """

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

        armature = blenderapi.scene().objects['human.armature']

        # human.applyMovement([sit_speed, 0, 0], True)
        armature['standUp'] = True
        armature.update()

        time.sleep(.5)
        """ Stops animating the human sitting down / standing up. """
        armature['standUp'] = False
        armature.update()

    @service
    def walk_away(self):
        """ Move human to left. """

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

        self.rotate(185)
        self.walk(3.5)
        self.rotate(85)
        self.walk(2)
        self.stop_animation()

    def walk(self, distance):
        """ Moves and animates the human. """

        frequency = 60
        walk_speed = 0.05
        duration = int(round(frequency * (distance / (frequency * walk_speed))))
        update_interval = 1 / frequency

        human = self.bge_object
        armature = blenderapi.scene().objects['human.armature']

        for i in range(duration):
            human.applyMovement([walk_speed, 0, 0], True)
            armature['movingForward'] = True
            armature.update()
            time.sleep(update_interval)

    def rotate(self, degrees):
        """ Rotates and animates the human. """

        frequency = 10
        duration = int(round(frequency * (abs(degrees) / 180)))
        rotation_speed = math.radians(180) / frequency # 180°/second

        if degrees < 0:
            rotation_speed = -rotation_speed
        
        update_interval = 1 / frequency

        human = self.bge_object
        armature = blenderapi.scene().objects['human.armature']

        for i in range(duration):
            human.applyRotation([0, 0, rotation_speed], True)
            armature['movingForward'] = True
            armature.update()
            time.sleep(update_interval)

    def stop_animation(self):
        """ Stops animating the human after walk or rotation. """
        
        time.sleep(.3)
        armature = blenderapi.scene().objects['human.armature']
        armature['movingForward'] = False
        for channel in armature.channels:     
            channel.rotation_mode = 6
            channel.joint_rotation = [0.0, 0.0, 0.0]
        armature.update()

    @service
    def walk_back(self):
        """ Move human to left. """

        if self.is_wa:
            self.is_wa = False
        else:
            return
        
        self.rotate(-180)
        self.walk(2)
        self.rotate(-85)
        self.walk(3.5)
        self.rotate(-5)
        self.stop_animation()

        self.is_ov = True
        self.is_oir = True

    @service
    def move(self, speed, rotation):
        """ Move the human. """

        human = self.bge_object

        if not human['Manipulate']:
            human.applyMovement([speed, 0, 0], True)
            human.applyRotation([0, 0, rotation], True)
        else:
            scene = blenderapi.scene()
            target = scene.objects['IK_Target_Empty.R']

            target.applyMovement([0.0, rotation, 0.0], True)
            target.applyMovement([0.0, 0.0, -speed], True)

    @service
    def look_around(self):
        """ Move the human head to look around. """

        if self.is_la:
            return

        if self.is_ag or self.is_gr:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        scene = blenderapi.scene()
        target = scene.objects['Look_Empty']

        f_speed_head = [0, 0, -0.1]
        # N = 10
        # for i in range(N):
        #    target.applyMovement([f_speed_head[0] / N, f_speed_head[1] / N, f_speed_head[2] / N], True)

        # look left up, left
        N = 10
        for i in range(N):
            target.applyMovement([0, -PI / 2 / N, PI / 2 / N], True)
            # target.applyMovement([0, 0, PI / 9 / N], True)
            time.sleep(0.1)
        for i in range(N):
            target.applyMovement([0, 0, -PI / 2 / N], True)
            # target.applyMovement([0, 0, -PI / 1.5 / N], True)
            time.sleep(0.05)
        time.sleep(1.5)
        for i in range(N):
            target.applyMovement([0, PI / N, 0], True)
            time.sleep(0.1)

        self.is_la = True
        self.is_ov = False

    def look_back(self):
        """ Move the human head to look back. """

        scene = blenderapi.scene()
        target = scene.objects['Look_Empty']

        # look right
        N = 5
        for i in range(N):
            target.applyMovement([0, -PI / 2 / N, 0], True)
            time.sleep(0.05)

        self.is_la = False
        self.is_ov = True

    @service
    def move_head(self, pan, tilt):
        """ Move the human head. """

        human = self.bge_object
        scene = blenderapi.scene()
        target = scene.objects['Target_Empty']

        if human['Manipulate']:
            return

        target.applyMovement([0.0, pan, 0.0], True)
        target.applyMovement([0.0, 0.0, tilt], True)

    def distance(self):
        ''' Distance between object and hand '''

        scene = blenderapi.scene()
        hand = scene.objects['IK_Target_Empty.R']
        obj = scene.objects['package1']

        vec1 = hand.worldPosition
        vec2 = obj.worldPosition

        dist = vec1 - vec2
        dist_value = math.sqrt(dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2])

        return dist_value

    @service
    def warn_robot(self):
        ''' corss both hands to warn robot'''

        if self.is_wr:
            return
        else:
            self.is_wr = True

        if self.is_la:
            self.look_back();

        if self.is_ag or self.is_gr:
            self.attempt_grasp_back()

        scene = blenderapi.scene()
        hand_r = scene.objects['IK_Target_Empty.R']
        hand_l = scene.objects['IK_Target_Empty.L']
        head = scene.objects['Look_Empty']
        back = scene.objects['Hips_Empty']
        obj = scene.objects['package1']

        ''' fetch hand '''
        vec1 = hand_r.localPosition
        # vec2 = dest.localPosition
        vec2 = obj.localPosition

        # fetch
        f_speed_head = [0, 0, 0]
        f_speed_right = [0.70, 0, 0.50]
        f_speed_left = [0.70, 0, 0.40]
        f_speed_back_rot = [10 * PI / 180, 0, 0]
        # fetch

        N = 25
        for i in range(N):
            head.applyMovement([f_speed_head[0] / N, f_speed_head[1] / N, f_speed_head[2] / N], True)
            hand_r.applyMovement([f_speed_right[0] / N, f_speed_right[1] / N, f_speed_right[2] / N], True)
            hand_l.applyMovement([f_speed_left[0] / N, f_speed_left[1] / N, f_speed_left[2] / N], True)
            back.applyRotation([f_speed_back_rot[0] / N, f_speed_back_rot[1] / N, f_speed_back_rot[2] / N], True)
            time.sleep(0.01)

        time.sleep(1)
        self.is_wr = True

    @service
    def warn_robot_back(self):
        scene = blenderapi.scene()
        hand_r = scene.objects['IK_Target_Empty.R']
        hand_l = scene.objects['IK_Target_Empty.L']
        head = scene.objects['Look_Empty']
        back = scene.objects['Hips_Empty']

        f_speed_head = [0, 0, 0]
        f_speed_right = [-0.70, 0, -0.50]
        f_speed_left = [-0.70, 0, -0.40]
        f_speed_back_rot = [-10 * PI / 180, 0, 0]
        # fetch

        N = 10
        for i in range(N):
            head.applyMovement([f_speed_head[0] / N, f_speed_head[1] / N, f_speed_head[2] / N], True)
            hand_r.applyMovement([f_speed_right[0] / N, f_speed_right[1] / N, f_speed_right[2] / N], True)
            hand_l.applyMovement([f_speed_left[0] / N, f_speed_left[1] / N, f_speed_left[2] / N], True)
            back.applyRotation([f_speed_back_rot[0] / N, f_speed_back_rot[1] / N, f_speed_back_rot[2] / N], True)
            time.sleep(0.01)

        # fetch hand back
        hand_l.localPosition = [0, 0.2, 0.82]
        hand_r.localPosition = [0, -0.2, 0.82]

        self.is_wr = False

    @service
    def grasp_animated(self):
        ''' grasp object '''

        if self.is_gr or self.is_ag:
            self.attempt_grasp_back()
        if self.is_la:
            self.look_back()
        if self.is_wr:
            self.warn_robot_back()
        self.is_gr = True

        """ Animating the human bending to grab object """

        frequency = 60
        grab_speed = 0.05
        duration = int(round(frequency * (1 / (frequency * grab_speed))))
        update_interval = 1 / frequency

        human = self.bge_object
        armature = blenderapi.scene().objects['human.armature']

        for i in range(duration):
            armature['bendToGrab'] = True
            armature.update()
            time.sleep(update_interval)

        time.sleep(.5)
        armature['bendToGrab'] = False
        armature.update()

    @service
    def grasp(self):
        ''' grasp object '''
        
        if self.is_gr or self.is_ag:
            self.attempt_grasp_back()
        if self.is_la:
            self.look_back()
        if self.is_wr:
            self.warn_robot_back()
        self.is_gr = True

        scene = blenderapi.scene()
        hand_r = scene.objects['IK_Target_Empty.R']
        hand_l = scene.objects['IK_Target_Empty.L']
        dest = scene.objects['IK_Pose_Empty.R']
        head = scene.objects['Look_Empty']
        back = scene.objects['Hips_Empty']

        obj = scene.objects['package1']

        ''' fetch hand '''
        vec1 = hand_r.localPosition
        # vec2 = dest.localPosition
        vec2 = obj.localPosition

        f_speed_head = [0, 0, -1]
        f_speed_right = [0.90, 0.1, -0.1]
        f_speed_left = [0.90, -0.1, -0.1]
        f_speed_back_rot = [45 * PI/180, 0, 0]
        f_speed_back_loc = [0, 0, 0.25]
        # fetch

        N = 25
        for i in range(N):
            head.applyMovement([f_speed_head[0] / N, f_speed_head[1] / N, f_speed_head[2] / N], True)
            hand_r.applyMovement([f_speed_right[0] / N, f_speed_right[1] / N, f_speed_right[2] / N], True)
            hand_l.applyMovement([f_speed_left[0] / N, f_speed_left[1] / N, f_speed_left[2] / N], True)
            back.applyRotation([f_speed_back_rot[0] / N, f_speed_back_rot[1] / N, f_speed_back_rot[2] / N], True)
            back.applyMovement([f_speed_back_loc[0] / N, f_speed_back_loc[1] / N, f_speed_back_loc[2] / N], True)
            time.sleep(0.01)

        time.sleep(1)

        self.is_gr = True

        # back
        dist = self.distance()

        # TODO: below the distance calculation will be fixed
        if dist < self.MIN_DIST:
            f_speed_head = [0, 0, 0.6]
            f_speed_right = [-0.40, 0.05, 0.15]
            f_speed_left = [-0.40, -0.05, 0.15]
            f_speed_back_rot = [-35 * PI / 180, 0, 0]
            f_speed_back_loc = [0, 0, -0.02]
            obj.setParent(hand_r)
            N = 50
            for i in range(N):
                head.applyMovement([f_speed_head[0] / N, f_speed_head[1] / N, f_speed_head[2] / N], True)
                hand_r.applyMovement([f_speed_right[0] / N, f_speed_right[1] / N, f_speed_right[2] / N], True)
                hand_l.applyMovement([f_speed_left[0] / N, f_speed_left[1] / N, f_speed_left[2] / N], True)
                back.applyRotation([f_speed_back_rot[0] / N, f_speed_back_rot[1] / N, f_speed_back_rot[2] / N], True)
                back.applyMovement([f_speed_back_loc[0] / N, f_speed_back_loc[1] / N, f_speed_back_loc[2] / N], True)
                obj.worldPosition = [hand_r.worldPosition[0] + 0.1, hand_r.worldPosition[1], hand_r.worldPosition[2] + 0.1]
                time.sleep(0.01)


            self.is_ho = True
            # putting the object into the container
            human = self.bge_object
            '''
            hand_r.localPosition = [0.25, -0.2, 0.8]
            hand_l.localPosition = [0.25, 0.2, 0.8]
            obj.worldPosition = [hand_r.worldPosition[0] - 0.2, hand_r.worldPosition[1] - 0.3, hand_r.worldPosition[2] + 0.3]
            '''
            N = 10
            # turn right 90 degree
            for i in range(N):
                human.applyRotation([0, 0, math.radians(-90) / N], True)
                obj.worldPosition = [hand_r.worldPosition[0] + 0.1, hand_r.worldPosition[1],
                                     hand_r.worldPosition[2] + 0.1]
                time.sleep(0.1)

            obj.worldPosition[2] -= 0.15
            obj.worldPosition[1] -= 0.15
            time.sleep(0.5)
            obj.removeParent()
            time.sleep(0.5)
            # standing direct after releasing the object
            f_speed_head = [0, 0, 0.3]
            f_speed_right = [-0.50, -0.15, -0.05]
            f_speed_left = [-0.50, 0.15, -0.05]
            f_speed_back_rot = [-10 * PI / 180, 0, 0]
            f_speed_back_loc = [0, 0, -0.23]
            obj.setParent(hand_r)
            N = 10
            for i in range(N):
                head.applyMovement([f_speed_head[0] / N, f_speed_head[1] / N, f_speed_head[2] / N], True)
                hand_r.applyMovement([f_speed_right[0] / N, f_speed_right[1] / N, f_speed_right[2] / N], True)
                hand_l.applyMovement([f_speed_left[0] / N, f_speed_left[1] / N, f_speed_left[2] / N], True)
                back.applyRotation([f_speed_back_rot[0] / N, f_speed_back_rot[1] / N, f_speed_back_rot[2] / N], True)
                back.applyMovement([f_speed_back_loc[0] / N, f_speed_back_loc[1] / N, f_speed_back_loc[2] / N], True)

                time.sleep(0.01)
            time.sleep(0.5)
            self.is_ho = False

            N = 5
            # turn left 90 degree
            for i in range(N):
                human.applyRotation([0, 0, math.radians(90) / N], True)
                time.sleep(0.1)

            human.worldPosition = [7.7, -1.25, 0]
            human.worldOrientation = [0, 0, -1.57]
            return True

        else:
            self.attempt_grasp_back()
            self.is_ho = False
            return False

    @service
    def attempt_grasp(self):
        ''' attempt to grasp object '''
        
        if self.is_gr or self.is_ag:
            self.attempt_grasp_back()
        if self.is_la:
            self.look_back()
        if self.is_wr:
            self.warn_robot_back()

        self.is_ag = True
        self.is_gr = True
        self.is_ho = False

        scene = blenderapi.scene()
        hand_r = scene.objects['IK_Target_Empty.R']
        hand_l = scene.objects['IK_Target_Empty.L']
        dest = scene.objects['IK_Pose_Empty.R']
        head = scene.objects['Look_Empty']
        back = scene.objects['Hips_Empty']

        obj = scene.objects['package1']

        ''' fetch hand '''
        vec1 = hand_r.localPosition
        # vec2 = dest.localPosition
        vec2 = obj.localPosition

        f_speed_head = [0, 0, -1]
        f_speed_right = [0.90, 0.1, -0.1]
        f_speed_left = [0.90, -0.1, -0.1]
        f_speed_back_rot = [45 * PI / 180, 0, 0]
        f_speed_back_loc = [0, 0, 0.25]
        # fetch

        N = 25
        for i in range(N):
            head.applyMovement([f_speed_head[0] / N, f_speed_head[1] / N, f_speed_head[2] / N], True)
            hand_r.applyMovement([f_speed_right[0] / N, f_speed_right[1] / N, f_speed_right[2] / N], True)
            hand_l.applyMovement([f_speed_left[0] / N, f_speed_left[1] / N, f_speed_left[2] / N], True)
            back.applyRotation([f_speed_back_rot[0] / N, f_speed_back_rot[1] / N, f_speed_back_rot[2] / N], True)
            back.applyMovement([f_speed_back_loc[0] / N, f_speed_back_loc[1] / N, f_speed_back_loc[2] / N], True)
            time.sleep(0.01)

        time.sleep(1)
        dist = self.distance()

        # TODO: below the distance calculation will be fixed
        if dist < self.MIN_DIST:
            f_speed_head = [0, 0, 0.6]
            f_speed_right = [-0.40, 0.05, 0.15]
            f_speed_left = [-0.40, -0.05, 0.15]
            f_speed_back_rot = [-35 * PI / 180, 0, 0]
            f_speed_back_loc = [0, 0, -0.02]
            obj.setParent(hand_r)
            N = 50
            for i in range(N):
                head.applyMovement([f_speed_head[0] / N, f_speed_head[1] / N, f_speed_head[2] / N], True)
                hand_r.applyMovement([f_speed_right[0] / N, f_speed_right[1] / N, f_speed_right[2] / N], True)
                hand_l.applyMovement([f_speed_left[0] / N, f_speed_left[1] / N, f_speed_left[2] / N], True)
                back.applyRotation([f_speed_back_rot[0] / N, f_speed_back_rot[1] / N, f_speed_back_rot[2] / N], True)
                back.applyMovement([f_speed_back_loc[0] / N, f_speed_back_loc[1] / N, f_speed_back_loc[2] / N], True)
                if i < 20:
                    obj.worldPosition = [hand_r.worldPosition[0] + 0.1, hand_r.worldPosition[1],
                                         hand_r.worldPosition[2] + 0.1]
                    time.sleep(0.01)
                else:
                    obj.removeParent()
                    time.sleep(0.01)

            obj.worldPosition = [7.7, -2.1, 0.80]
            self.attempt_grasp_back()
            time.sleep(0.5)
            self.is_ho = False
            return True

        else:
            self.attempt_grasp_back()
            self.is_ho = False
            return False


    @service
    def attempt_grasp_back(self):
        scene = blenderapi.scene()
        hand_r = scene.objects['IK_Target_Empty.R']
        hand_l = scene.objects['IK_Target_Empty.L']
        target = scene.objects['Look_Empty']    
        # back
        hand_r.localPosition = [0, -0.2, 0.82]
        hand_l.localPosition = [0, 0.2, 0.82]
        N = 5
        for i in range(N):
            target.applyMovement([0, 0, -0.], True)
            time.sleep(0.1)

        self.is_ag = False
        self.is_gr = False

    @service
    def move_hand(self, diff, tilt):
        """ Move the human hand (wheel).

        A request to use by a socket.
        Done for wiimote remote control.
        """

        human = self.bge_object
        if human['Manipulate']:
            scene = blenderapi.scene()
            target = scene.objects['IK_Target_Empty.R']
            target.applyMovement([diff, 0.0, 0.0], True)

    @service
    def toggle_manipulation(self):
        """ Change from and to manipulation mode.

        A request to use by a socket.
        Done for wiimote remote control.
        """

        human = self.bge_object
        scene = blenderapi.scene()
        hand_target = scene.objects['IK_Target_Empty.R']
        head_target = scene.objects['Target_Empty']

        # '''
        if human['Manipulate']:
            human['Manipulate'] = False
            # Place the hand beside the body
            hand_target.localPosition = [0.0, -0.3, 0.8]
            head_target.setParent(human)
            head_target.localPosition = [1.3, 0.0, 1.7]
        else:
            human['Manipulate'] = True
            head_target.setParent(hand_target)
            # Place the hand in a nice position
            hand_target.localPosition = [0.6, 0.0, 1.4]
            # Place the head in the same place
            head_target.localPosition = [0.0, 0.0, 0.0]
        # '''

        return human['Manipulate']

    @service
    def manipulation_state(self):
        """return state of human['Manipulate'] """
        human = self.bge_object

        return human['Manipulate']

    def default_action(self):
        """ Main function of this component. """
        pass

