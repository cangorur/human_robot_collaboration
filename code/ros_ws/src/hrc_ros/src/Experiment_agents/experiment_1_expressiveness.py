#!/usr/bin/env python
# This script subscribes to the observation_agent/observation_update topic and writes the POMDP observations to a .csv file. 
# It can either be used with a recorded bagfile or it can run during the experiment to record the stimuli. 

import rospy
from std_msgs.msg import *
from hrc_ros.srv import *
from hrc_ros.msg import *
import pandas as pd

time_cnt = 0

f = open('bag_extraction.csv','w')
 #Give your csv text here.

def callback(msg):
    global time_cnt
    time_cnt = time_cnt + 1
    print(time_cnt)
    if (time_cnt == 1):
        f.write(str(msg.mapped_observation_pomdp))
        print("1st occurence")
    else:
        f.write("," + (str(msg.mapped_observation_pomdp)))
        print("other occurences")
        
def listener():

    rospy.init_node('rosbag_extractor_node', anonymous=True)

    #rospy.Subscriber("/observation_agent/observation_update", hrc_ros.msg.ObsUpdateMsgIE  , callback)

   # ros::Subscriber dobot_grasp_sub = nh.subscribe("/robot_motion_agent/dobot_grasp",1, graspCallback);  
   # ros::Subscriber dobot_cancel_sub = nh.subscribe("/robot_motion_agent/dobot_cancel",1, cancelCallback); 
#	ros::Subscriber dobot_plan_sub = nh.subscribe("/robot_motion_agent/dobot_plan",1, planCallback);  
#	ros::Subscriber dobot_idle_sub = nh.subscribe("/robot_motion_agent/dobot_idle",1, idleCallback); 
#	ros::Subscriber dobot_point_sub = nh.subscribe("/robot_motion_agent/dobot_point",1, pointCallback); 
#	ros::Subscriber dobot_resume_sub = nh.subscribe("/robot_motion_agent/dobot_resume",1, resumeQueueCallback); 
#	ros::Subscriber dobot_gohome_sub = nh.subscribe("/robot_motion_agent/dobot_gohome",1, returnHomeCallback); 
#	ros::Subscriber dobot_calibration_sub = nh.subscribe("/robot_motion_agent/dobot_calibration",1, gotoCallibrationCallback); 
   
    pub_grasping = rospy.Publisher("/robot_motion_agent/dobot_grasp", std_msgs.msg.Bool , queue_size=1)
    pub_planning = rospy.Publisher("/robot_motion_agent/dobot_plan", std_msgs.msg.Bool , queue_size=1)
    pub_cancel   = rospy.Publisher("/robot_motion_agent/dobot_cancel", std_msgs.msg.Bool , queue_size=1)
    pub_pointing = rospy.Publisher("/robot_motion_agent/dobot_point", std_msgs.msg.Bool , queue_size=1)
    pub_go_home  = rospy.Publisher("/robot_motion_agent/dobot_gohome", std_msgs.msg.Bool , queue_size=1)
    pub_idle     = rospy.Publisher("/robot_motion_agent/dobot_idle", std_msgs.msg.Bool , queue_size=1)



    # publisher to publish grasp colour - this is needed because otherwise the conveyor is recognized as empty and no grasp will be done 
    # object is set to red for now 
    pub_object_to_grasp_colour = rospy.Publisher("/object_tracking/object_tograsp_colour", hrc_ros.msg.ObjectGraspColourMsg,queue_size=1)
    object_grasp_msg = ObjectGraspColourMsg()
    object_grasp_msg.object_colour = 1
    pub_object_to_grasp_colour.publish(object_grasp_msg)

    rate = rospy.Rate(10) # 10 Hz
    print("Specify gesture to be displayed [1 - 7] ")
    print( " # Sequence:")
    print( "  0 - go home ")  
    print( " # 1 - grasping         ")       
    print( " # 2 - Planning V1      ")    
    print( " # 3 - Cancel Planning  ") 
    print( " # 4 - Pointing V1      ")
    print( " # 5 - Pointing V2      ")
    print( " # 6 - IDLE             ")
    print( " # 7 - Planning V2      ")
    gesture_cnt = 0 

    while not rospy.is_shutdown():
        if (gesture_cnt < 2):
            pub_object_to_grasp_colour.publish(object_grasp_msg)
        gesture = int(raw_input('Enter gesture:'))
        print(gesture)
        
        gesture_cnt += 1
        if (gesture_cnt < 2):
            pub_object_to_grasp_colour.publish(object_grasp_msg)

        # ######  Publish the actions to DobotWorker agent 
        

        print( " # Sequence:")
        print( "  0 - go home ")  
        print( " # 1 - grasping         ")       
        print( " # 2 - Planning V1      ")    
        print( " # 3 - Cancel Planning  ") 
        print( " # 4 - Pointing V1      ")
        print( " # 5 - Pointing V2      ")
        print( " # 6 - IDLE             ")
        print( " # 7 - Planning V2      ")
        if (gesture < int(5)):   # expression version 1 
            # set parameter to version 1 
            rospy.set_param("/dobot_expression_version", 1)

            
            if gesture == int(0): # go to home location 
                print("go to home location")
                pub_go_home.publish(True)

            if gesture == int(1):  # grasping 
                print("publish grasping")
                pub_grasping.publish(True)
            
            if gesture == int(2):  # planning V1  
                print("publish planning - V1")
                pub_planning.publish(True)

            if gesture == int(3):  # cancel  
                print("publish cancelling ")
                pub_cancel.publish(True)

            if gesture == int(4):  # pointing V1  
                print("publish pointing - V1 ")
                pub_pointing.publish(True)


        elif(gesture >= int(5) ): 
            # set parameter to version 2 
            rospy.set_param("/dobot_expression_version", 2)
            
            if gesture == int(5): # pointing V2 
                print("publish pointing - V2")
                pub_pointing.publish(True)

            if gesture == int(6): # idle 
                print("publish idle")
                pub_idle.publish(True)

            if gesture == int(7): # planning V2
                print("publish planning - V2")
                pub_planning.publish(True)
        
        #rospy.spin()
        rate.sleep()



if __name__ == '__main__':
    listener()
