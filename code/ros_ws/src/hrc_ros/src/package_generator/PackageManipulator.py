#!/usr/bin/env python3

from hrc_ros.srv import *
import rospy
import pymorse 

def handleMovePackage(req):
    success = True
    try:
        with pymorse.Morse() as sim:
            sim.rpc('simulation', 'set_object_position', req.package_id, [req.x, req.y, req.z])
    except:
        success = False
    rospy.logdebug("[Package Manipulator]: MoveNewPackage {} success: {}!".format(req.package_id, success))
    return MoveNewPackageResponse(success)

def addMovePackageServer():
    s = rospy.Service('~move_new_package', MoveNewPackage, handleMovePackage)

if __name__ == "__main__":
    rospy.init_node('package_manipulator')
    addMovePackageServer()
    rospy.loginfo("PackageManipulator ready!")
    rospy.spin()
