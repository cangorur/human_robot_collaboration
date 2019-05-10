## Building the project 

* Before checking out the project you should create a catkin workspace. 

```
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/
 catkin_make
```

* Navigate to the catkin_ws/src directory and clone the following two ros packages:  
	* **objectTracking_ROS:** ```git clone https://gitlab.tubit.tu-berlin.de/f308910/objectTracking_ROS.git```
		* after that checkout the **publish_actions_POMDP_test** branch: git checkout ```git checkout publish_actions_POMDP_test``` 
	* **hrc_industry**: ```git clone https://gitlab.tubit.tu-berlin.de/app-ras/hrc_industry.git```
		* after that checkout the **interaction_experiment_v2** branch: ```git checkout interaction_experiment_v2```

* Navigate to the *catkin_ws/* directory and build the packages. You can either: 
	* Build both packages at once: ``` catkin_make ```
	* Build the packages separately: ```catkin_make --pkg hrc_industry ``` OR ```catkin_make --pkg object_tracking```

=========================================================================
## Starting Dobot and the conveyor belt:

### on pc:
 - minh@minh-ThinkPad-W530:~/ba/catkin_ws$ 
 - ```source devel/setup.bash```
 
 - setup the remote ros master:
```
 export ROS_MASTER_URI=http://130.149.232.237:11311
 export ROS_HOSTNAME=130.149.232.210
```
 - Or use the alias **remote_ros_master**  

### RaspberryPi1 dobot arm control: 
 - ```ssh ubuntu@130.149.232.237```   in 2 terminals  
 - password: ubuntu

 - Turn on the Raspberry pi of the dobot and turn on the dobot arm. 
 - wait until dobot beeps

#### on one terminal - launch dobot physical layer and middle layer:
 
 - ```source dobot-conveyor-unit/DOBOT_UNIT/devel/setup.bash && roslaunch dobot dobot.launch```  or use the alias **setup_dobot_physical_1** on the raspberry pi
 - Note: dobot will drive to a calibration position on the far left 

#### wait until dobot stops moving then in another terminal ( can also be launched locally on PC) - launch dobot and conveyor app layer:
 
 - ```source dobot-conveyor-unit/CHARIOT/devel/setup.bash && roslaunch dobot app.launch```   or use the alias **setup_dobot_app_2** on the raspberry pi 


### RaspberryPi2 conveyor belt:
 - ```ssh ubuntu@10.0.8.193```
 - password:ubuntu

 - turn on the conveyor belt Raspberry Pi and also switch on the power supply for the conveyor motor

#### one terminal - launch conveyor physical layer and middle layer:
- ```source dobot-conveyor-unit/CONVEYOR_UNIT/devel/setup.bash && roslaunch dobot motor.launch```


#### in a second terminal - launch the conveyor_control app layer :   
- ```source dobot-conveyor-unit/CHARIOT/devel/setup.bash && roslaunch dobot app.launch```

=========================================================================
## Launch all nodes 

**NOTE:** In each terminal that launches nodes, the remote rosmaster should be configured as follows: 
```
export ROS_MASTER_URI=http://130.149.232.237:11311 && export ROS_HOSTNAME=130.149.232.210
```

* Launch the **rule_monitor_agent** on a PC that is connected to a big screen. This node should be run in a maximised terminal and should be always in the foreground, as it will display the current task rules. Make sure you configure the remote ros master as described above.
```
rosrun hrc_ros rule_monitor_agent
```

* There is a **toplevel launch file** that you can launch like this 
```
roslaunch hrc_ros IE_experiment.launch
```
* this launches the **openni2 driver**, all nodes of the **object_tracking** package as well as the **hrc_ros** packages. The exact nodes are: 
	* openni2 driver and the respective nodes
	* object_tracking nodes: 
		* /activitiy_recognition_continousHMM
		* /obj_track
		* /tray_observer
	* hrc_ros nodes: 
		* /dobot_worker_agent
		* /observation_agent_IE
		* /robot_motion_agent 
		* /task_manager_IE
	

* the launch file makes use of the timed_roslaunch package, if you get an error install it by 
```
sudo apt install ros-kinetic-timed-roslaunch
source /opt/ros/kinetic/setup.bash
```

* If you want to have **more control** over the nodes you can also launch them manually. Additionally there are 2 launchfiles that will either launch the hrc_nodes or the object_tracking nodes 

	* **hrc_industry nodes:** ``` roslaunch hrc_ros hrc_nodes.launch ```

	* **object_tracking nodes** ``` roslaunch hrc_ros object_tracking_nodes.launch ``` 


=========================================================================
## Start the experiment: 
Call the following service to start an experiment. If the **rule_monitor_node** is running it will show the current task rules for a certain amount of time ad will turn blank afterwards, this indicates that the human can start with the experiment and the DESPOT will be started at the same time. 

```
rosservice call /task_manager_IE/new_scenario_request
```

**NOTE:**  robot_motion_agent sometimes does not terminate cleanly. You can kill it by: ```killall robot_motion_agent```


## Services to manually start and control the system 

### Start the conveyor belt: 
```
rosservice call /conveyor_control_app/inOprConveyorControl
```

## System monitoring 
To monitor which observations are sent to the DESPOT and wich action is taken by the robot the following topics can be inspected 
- /experiment_monitoring/robot_action_taken_pub   |   published by /robot_motion_agent      | published after DESPOT informed
- /observation_agent/observation_update		  |   published by /observation_agent_IE    | published when sending obs to DESPOT
All relevant information is also available in a combined topic: 
- /task_manager/task_status			| published by /task_manager		  | combined topic that includes all relevant information 

------------------------------------------------------------------------------------


## Experiment 1 - Robot Expressiveness 

* configure the RaspberryPi1 with the dobot arm control nodes as described above.
* run the dobot_worker node ''' rosrun hrc_ros dobot_worker_agent'''  (configure the remote rosmaster in the terminal first as described above 
* run the experiment node that is used to trigger the dobot expressions '''rosrun hrc_ros experiment_1_expressiveness.py'''
	* you can specify numbers from 11 to 27. The numbers are compiled like this [experiment_part 1/2, expresspin from 1-7]

|Number for experiment1 node |Experiment part				|Expression	| 
|---			     |---									|---		|	 
|11			     	 | 1 - guess expression					|			|			
|12			     	 |  1 - guess expression				|			|
|13			     	 | 1 - guess expression					|			|
|14			         | 1 - guess expression					|			|
|15 			     | 1 - guess expression					|			|
|16			         | 1 - guess expression					|			|
|17			         | 1 - guess expression					|			|
| 			         |										|			|
|21			         | 2 - select from given expressions	|			|
|22			         | 2 - select from given expressions	|			|
|23 			     | 2 - select from given expressions	|			|
|24			     	 | 2 - select from given expressions	|			|
|25			     	 | 2 - select from given expressions	|			|
|26 			     | 2 - select from given expressions	|			|
|27			     	 | 2 - select from given expressions	|			|
	
	
------------------------------------------------------------------------------------


## Parameters to configure the experiment and dobot dynamically 

### rosparameters 
Can be set by 
`` rosparam set /parameter_name ``

- **/dobot_interrupt_immediately: default=false** If set to true, dobot will react to human warnings directly, this will circumvent the DESPOT decision making in the case of a human warning (all other observations will still go to DESPOT first) 
- **/noDobot: default=false** if set true, the dobot services will not be called. Instead a time delay will simulate the dobot behaviour. This can be used to test the system even if dobot is not connected/available. 
- **/dobot_expression_version: default=2** Valid values are 1 and 2. This can be used to switch between two different robot expressiveness versions. V1 = without shared attention and quite simple gesture | V2 = based on shared/joint attention and more elaborate gestures. Point and Planning action are affected. 

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



