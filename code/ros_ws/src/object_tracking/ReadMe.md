tested with: ubuntu 16.04, opencv 4.0, c++11 std, ros kinetic

opencv: "Getting the Cutting-edge OpenCV from the Git Repository" https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html
should be version 4.0

## Setting it up

for branches "demo" and "master":
dobot project has to be built to work since it is uses services and messages from there

also do and replace addresses accordingly:
```
export ROS_MASTER_URI=http://130.149.232.237:11311
export ROS_HOSTNAME=130.149.232.210
```

for all:

ros node to be built with catkin_make:

in your catkin workspace (create one if you didnt,yet) ie. ~/catkin_ws

load files (msg, settings, src, srv, CMakeLists.txt, package.xml) to ~/catkin_ws/src/object_tracking


in ~/catkin_ws/ do
```
catkin_make
```

source the setup.bash from your catkin workspace:
in ~/catkin_ws/devel/ do
```
source setup.bash 
```

## Running it

first run openni2 in a terminal (should already be installed with ros kinetic and kinetic being sourced):
```
roslaunch openni2_launch openni2.launch depth_registration:=true
```

then 
```
roslaunch object_tracking tracking.launch
```

or INSTEAD of running tracking.launch do in another 3 terminals:

```
rosrun object_tracking feature_processer 
```
```
rosrun object_tracking tray_observer
```
```
rosrun object_tracking obj_track
```

~/catkin_ws/src/object_tracking/settings/settings.txt will get loaded at node obj_track startup to decide to launch in calibration or tracking mode


## Guidelines:

- no sunlight; shut the blinds and use room lights
- only packages gloves and containers should be  green,blue,red,yellow colored
- wear long sleeves as not to detect skin as a package (depending on skin color)
- make sure that the robot is not blocking the boxes from camera view when launching tracking, since boxes are set in the system by first ~5 frames at start up

Human constraints when tracking:
- grasp package only with 2-3 fingers to ensure the hand is not hiding the package from the camera
- use slow grasping motions and dont release package at height > 1.5packages
- after puting package in a container: wait with hand 0.25 seconds in front of conveyor belt before picking up next package
->this resets the kalman from last package to conveyor package in case those have the same color, subject to change though

Calibration Guidelines:
- have all objects present when calibrating (red,green,blue packages/container and yellow glove)
- put packages in container and on conveyor belt (each container has to have each package color inside)
- calibrate red in moving and not moving state
- when calibrating packages, also have something throw a shadow on a part of the containers
 -> otherwise container could be seen as package when moving above container and having the hand shadow

How to Calibrate:
- edit /settings/settings.txt to set calib to true (false for tracking mode)
- do "rosrun object_tracking obj_track" (or restart when it is already running)
- use windows "blured after threshholding", "contour", "rgb image" and "Trackbars" and move the trackbars to get HSV values for the objects, print the values by pressing o
- update settings.txt and set calib to false when done

Hotkeys in calibration mode:
- q - toggle between red, green, blue packages
- w - toggle between red, green, blue containers
- m - toggle red non-moving value
- e - show yellow glove
- o - print current calib hsv values in console (formated for easy copy paste into settings.txt)
- r - reset hsv calib values
- t - toggle conveyor color (outdated)
- 1 - load settings.txt
