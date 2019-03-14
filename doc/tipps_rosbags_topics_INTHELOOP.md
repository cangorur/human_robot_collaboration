## Some tips for working with rosbags 

### replaying depth bags with openni2 
roslaunch openni2_launch openni2.launch load_driver:=false depth_registration:=true
rosbag play --clock dataset_upd.bag

### filtering rosbags and export only selected topics 
rosbag filter my.bag only-tf.bag "topic in ['/camera/depth_registered/camera_info' , '/camera/depth_registered/image_raw' , '/camera/rgb/camera_info' , '/camera/rgb/image_raw' ]"

### running the action_recognition part 
roslaunch object_tracking tracking.launch

#### topics relevant for POMDP -> tray_update and action_recognized 
- /experiment_monitoring/robot_action_taken_pub   |   published by /robot_motion_agent      | published after DESPOT informed
- /observation_agent/observation_update		|   published by /observation_agent_IE    | published when sending obs to DESPOT
- rosbag filter my.bag only-tf.bag "topic in [ '/experiment_monitoring/robot_action_taken_pub', '/observation_agent/observation_update' , '/camera/rgb/camera_info' , '/camera/rgb/image_raw' ]"
- rosbag filter my.bag only-tf.bag "topic in [ '/experiment_monitoring/robot_action_taken_pub', '/observation_agent/observation_update']"


### topics published by action recognition
- /object_tracking/publish_tray_update		| The observation_agent must be running 
- /camera_agent/motion_features 			| motion feature only no classes
- /action_recognition/publish_action_recognized   | classes published byHMMGMM(5.py) | published when service is issued to obs_agent


____________________________________________________________________________________
## setting it up:


### on pc:
 - minh@minh-ThinkPad-W530:~/ba/catkin_ws$ 
 - source devel/setup.bash
 - export ROS_MASTER_URI=http://130.149.232.237:11311
 - export ROS_HOSTNAME=130.149.232.210

 - Or use the alias remote_ros_master 

### RaspberryPi1 dobot arm control: 
 - ssh ubuntu@130.149.232.237
 - password: ubuntu

 - Turn on the Raspberry pi of the dobot and turn on the dobot arm. 

#### one terminal:
 
 - source hrc_integration/devel/setup.bash && roslaunch dobot_arm_lower dobot.launch


#### wait for robot to stop moving then in another terminal:
 
 - source hrc_integration/devel/setup.bash && roslaunch dobot_chariot app.launch



### RaspberryPi2 conveyor belt:
 - ssh ubuntu@10.0.8.193
 - password:ubuntu

#### one terminal:
- source dobot_conveyor_unit_polishing/CONVEYOR_UNIT/devel/setup.bash && roslaunch dobot motor.launch

- in a second terminatl:  ONLY run if you are not running dobot as well
- source dobot_conveyor_unit_polishing/CHARIOT/devel/setup.bash && roslaunch dobot app.launch



### Start the conveyor belt: 

 - rosservice call /conveyor_control_app/inOprConveyorControl