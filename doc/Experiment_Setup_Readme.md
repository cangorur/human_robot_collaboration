## Starting the Experiment setup:

### on pc:
 - minh@minh-ThinkPad-W530:~/ba/catkin_ws$ 
 - source devel/setup.bash
 - export ROS_MASTER_URI=http://130.149.232.237:11311
 - export ROS_HOSTNAME=130.149.232.210

 - Or use the alias remote_ros_master 

### RaspberryPi1 dobot arm control: 
 - ssh ubuntu@130.149.232.237   in 2 terminals 
 - password: ubuntu

 - Turn on the Raspberry pi of the dobot and turn on the dobot arm. 
 - wait until dobot beeps

#### on one terminal:
 
 - source hrc_integration/devel/setup.bash && roslaunch dobot_arm_lower dobot.launch
 - Note: dobot will drive to a calibration position on the far left 

#### wait until dobot stops moving then in another terminal:
 
 - source hrc_integration/devel/setup.bash && roslaunch dobot_chariot app.launch


### RaspberryPi2 conveyor belt:
 - ssh ubuntu@10.0.8.193
 - password:ubuntu

#### one terminal:
- source dobot_conveyor_unit_polishing/CONVEYOR_UNIT/devel/setup.bash && roslaunch dobot motor.launch

- in a second terminal:  ONLY run if you are not running dobot as well 
- source dobot_conveyor_unit_polishing/CHARIOT/devel/setup.bash && roslaunch dobot app.launch


## Launch all nodes 
- there is a toplevel launch file that you can launch like this 
```
roslaunch hrc_ros IE_experiment.launch
```
- this launches the **openni2 driver**, all nodes of the **object_tracking** package as well as the **hrc_ros** packages
- the launch file makes use of the timed_roslaunch package, if you get an error install it by 
```
sudo apt install ros-kinetic-timed-roslaunch
source /opt/ros/kinetic/setup.bash
```


## Services to manually start and control the system 

### Start the conveyor belt: 
```
rosservice call /conveyor_control_app/inOprConveyorControl
```




____________________________________________________________________________________

## Some tips for working with rosbags 

### replaying depth bags with openni2 
roslaunch openni2_launch openni2.launch load_driver:=false depth_registration:=true
rosbag play --clock dataset_upd.bag

### reducing resolution of asus xtion 
rosrun dynamic_reconfigure dynparam set /camera/driver color_mode 8
or in launchfile: <param name="camera/driver/color_mode" value="8" /> 

### running the action_recognition part 
roslaunch object_tracking tracking.launch

#### topics relevant for POMDP -> tray_update and action_recognized 
- /experiment_monitoring/robot_action_taken_pub   |   published by /robot_motion_agent      | published after DESPOT informed
- /observation_agent/observation_update		|   published by /observation_agent_IE    | published when sending obs to DESPOT
- /task_manager/task_status			| published by /task_manager		  | combined topic that includes all relevant information 
- rosbag filter my.bag only-tf.bag "topic in [ '/experiment_monitoring/robot_action_taken_pub', '/observation_agent/observation_update' , '/task_manager/task_status', '/camera/rgb/camera_info' , '/camera/rgb/image_raw' ]"
- rosbag filter my.bag only-tf.bag "topic in [ '/experiment_monitoring/robot_action_taken_pub', '/observation_agent/observation_update', '/task_manager/task_status']"

### topics published by action recognition
- /object_tracking/publish_tray_update		| The observation_agent must be running 
- /camera_agent/motion_features 			| motion feature only no classes
- /action_recognition/publish_action_recognized   | classes published byHMMGMM(5.py) | published when service is issued to obs_agent


### Convert rosbags to mp4 videos 
- See here for tipps: http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data
- change path to your bag in the launchfile convert_bag_jpeg.launch
- roslaunch hrc_ros convert_bag_jpeg.launch   => jpegs will be stored to ~/.ros/ 
- copy images to working_directory: mv ~/.ros/frame.jpg working_directory
- change to working_directory -> for mp4 video tryout different framerates! (either 30 or 15 should work fine) 
- ffmpeg -framerate 25 -i frame%04d.jpg -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p output.mp4



