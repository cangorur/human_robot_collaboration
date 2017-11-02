Human-Robot Collaboration on an Assembly Line (Industrial Scenario)
============

by Orhan Can Görür (orhan-can.goeruer@dai-labor.de)

Below are the instructions to succesfully install the current HRC simulation on a conveyor belt (also referring to the other notes) for pick up and place/store scenario. The code also involves autonomous decision-making and actuator models for both human and the robot (pr2). The goal is to test these DM algorithms for the robot in response to human behaviors which can resemble a tired, distracted, thinker, stubborn, beginner and expert human worker types. These situations and randomness in human actions constitute difficult cases for the robot to respond properly and reliably.

---

## Prerequisites
- MORSE

Install Morse 1.3 - STABLE.

Overwrite your supervision file (of morse installed) with the file from the repository (under the project folder):
 
```
sudo cp ./supervision_services.py <morse_install_path>/lib/python3/dist-packages/morse/services/supervision_services.py
```

- ROS

The code is tested with ROS Kinetic on Ubuntu 16.04 machines.
No special ROS packages are needed apart from those which come with a standard installation of ROS.
The only other dependency are the *Boost libraries* which need to be installed.

---

## Installation:

```
cd code/ros_ws
catkin_make install
```
NOTE: If it gives an error (probably not able to find msg adn srv headers), this is a bug. Reinvoke catkin_make install. If still an error, navigate to build folder generated already by catkin_make, and invoke "make" inside it to force the compile.

Now we will compile the DESPOT packages each tailored for different model executions in real-time: MDP for human and robot, POMDP for robot decision-making (autonomous):
```
cd code/despot_MDP_human
mkdir build
cd build
cmake ..
make

cd code/despot_MDP_robot
mkdir build
cd build
cmake ..
make

cd code/despot_POMDP_robot
mkdir build
cd build
cmake ..
make
```

We need to overwrite some morse source libraries
Warning: Before we start overwriting some of the morse source files, you may want to create their copies!
```
cd code/hrc_morse/src
# We will replace human.py class with our version:
sudo cp human.py <morse_installation_path>/lib/python3/dist-packages/morse/robots/
# We will replace pr2.py class with our version:
sudo cp pr2.py <morse_installation_path>/lib/python3/dist-packages/morse/robots/
# We will replace main.py of blender with our version (a bug fix for overlayed objects):
sudo cp main.py <morse_installation_path>/lib/python3/dist-packages/morse/blender/
# Add the conveyor belt as a robot to open ros service interfaces to be able to control its operation
sudo cp conveyor.py <morse_installation_path>/lib/python3/dist-packages/morse/builder/robots/
sudo cp conveyor_srv.py <morse_installation_path>/lib/python3/dist-packages/morse/robots/
```
Now overwriting human and pr2 robot model blends and adding our conveyor belt design:
```
cd code/hrc_morse/data
# We will replace human.blend with our updated design:
sudo cp human.blend <morse_installation_path>/share/morse/data/robots/
# We will replace pr2.blend with our updated design
sudo cp pr2.blend <morse_installation_path>/share/morse/data/robots/
# Add the conveyor belt blend drawing to the morse project
sudo cp conveyor.blend <morse_installation_path>/share/morse/data/robots/
```

make surethat your builder script ("hrc_industry.py" in our case) has this line to import conveyor belt properly:
```
from morse.builder.robots.conveyor import *
```

To add more conveyor belts, add these below to your builder script:
```
Tip to tip (longer band)
>conveyor1 = Conveyor()
>conveyor1.translate(x, y, z)
>conveyor2 = Conveyor()
>conveyor2.translate(x, y-1.8, z)

L shaped conveyor belts:
>conveyor1 = Conveyor()
>conveyor1.translate(x, y, z)
>conveyor2 = Conveyor()
>conveyor2.translate(x + 1.4, y - 0.6, z)
>conveyor2.rotate(z = pi/2) # if math is not imported, then z = 1.57
```

## Running
```
cd code/
morse import hrc_morse # --> this is to import the folder as a morse project

source ros_ws/devel/setup.bash # should source for both the morse and ros terminals
roscore
morse run hrc_morse hrc_scenario.py
roslaunch hrc_ros hrc.launch
```
Every process is run by a rosservice call. Run "rosservice list" to see all available interfaces ! You can simply run each by typing "rosservice call /human/walk_away"
To run the system:
```
rosservice call /hrc_task_manager/new_scenario_request
```
This initiates the conveyor belts and a package. Package stops in between human and the robot and two terminals pops out. One terminal is for selecting human states to guide human actions (see human states from 0 to 10) under code/models/human_models (any model has the same human states). For the other terminal, it is robot's decision-making and it operates automatically, just for monitoring purposes. Once a package is stored/failed and fell into the unprocessed container, a scenario ends and you should manually close the two terminals popped out (this will be fixed in the future). A reinvoke of new scenario request will initiate another scenario. Just mind that as the task numbers increases human gets more tired/distracted/thinker leading him to fail to grasp or not attending.

## Models

All the predesigned human and robot models can be found under code/models.

Which model (human and robot types) to run initially can be selected under code/configs/scenario_config.json
Just replace the type fields under the json file with the ones below:
human types: expertise: "beginner / expert" || type:mood:"stubborn / thinker / distracted / tired" # Note that in time human is assigned randomly but more likely with distracted and tired types automatically.
robot type: "proactive / reactive"

