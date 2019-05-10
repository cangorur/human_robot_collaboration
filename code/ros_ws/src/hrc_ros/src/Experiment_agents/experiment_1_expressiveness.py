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

    rospy.Subscriber("/observation_agent/observation_update", hrc_ros.msg.ObsUpdateMsgIE  , callback)

   # ros::Subscriber dobot_grasp_sub = nh.subscribe("/robot_motion_agent/dobot_grasp",1, graspCallback);  
   # ros::Subscriber dobot_cancel_sub = nh.subscribe("/robot_motion_agent/dobot_cancel",1, cancelCallback); 
#	ros::Subscriber dobot_plan_sub = nh.subscribe("/robot_motion_agent/dobot_plan",1, planCallback);  
#	ros::Subscriber dobot_idle_sub = nh.subscribe("/robot_motion_agent/dobot_idle",1, idleCallback); 
#	ros::Subscriber dobot_point_sub = nh.subscribe("/robot_motion_agent/dobot_point",1, pointCallback); 
#	ros::Subscriber dobot_resume_sub = nh.subscribe("/robot_motion_agent/dobot_resume",1, resumeQueueCallback); 
#	ros::Subscriber dobot_gohome_sub = nh.subscribe("/robot_motion_agent/dobot_gohome",1, returnHomeCallback); 
#	ros::Subscriber dobot_calibration_sub = nh.subscribe("/robot_motion_agent/dobot_calibration",1, gotoCallibrationCallback); 
   
    pup_grasping = rospy.Publisher("/robot_motion_agent/dobot_grasp", std_msgs.msg.Bool , queue_size=1)

    rate = rospy.Rate(10) # 10 Hz
    print("Specify gesture to be displayed [11 - 27] ")
    while not rospy.is_shutdown():
        gesture = int(raw_input('Enter gesture:'))
        print(gesture)
        

        if (gesture < int(20)):
            # set parameter to version 1 
            
            if gesture == int(11):
                pup_grasping.publish(True)

        elif(gesture >= int(20) ):
            print("in bigger")
            pup_grasping.publish(False)
            # set parameter to version 2
            

        rate.sleep()



if __name__ == '__main__':
    listener()
