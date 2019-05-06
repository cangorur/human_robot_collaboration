#!/usr/bin/env python
# This script subscribes to the observation_agent/observation_update topic and writes the POMDP observations to a .csv file. 
# It can either be used with a recorded bagfile or it can run during the experiment to record the stimuli. 

import rospy
from std_msgs.msg import String
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

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
