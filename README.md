Human-Robot Collaboration on an Assembly Line (Industrial Scenario)
============

by Orhan Can Görür (orhan-can.goeruer@dai-labor.de)

Below are the instructions to succesfully install the current HRC simulation on a conveyor belt (also referring to the other notes) for pick up and place/store scenario. The code also involves autonomous decision-making and actuator models for both human and the robot (pr2). The goal is to test these DM algorithms for the robot in response to human behaviors which can resemble a tired, distracted, thinker, stubborn, beginner and expert human worker types. These situations and randomness in human actions constitute difficult cases for the robot to respond properly and reliably.

For the students, the architecture drawing at the bottom of the page shows the possible improvement points and how they can interfere with the process. For more questions, please contact the mail given above.

---

## Prerequisites
- MORSE

Install Morse 1.3 - STABLE. Please refer to APP-RAS W3 slides

After the installation, extend `PYTHONPATH` with the MORSE installation path. Just puth the below line to `.bashrc` file.
```
export PYTHONPATH=${PYTHONPATH}:/opt/lib/python3/dist-packages
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
# Overwrite your supervision file (of morse installed) with the file from the repository (under the project folder):
sudo cp supervision_services.py <morse_install_path>/lib/python3/dist-packages/morse/services/supervision_services.py
# We will replace human.py class with our version:
sudo cp human.py <morse_installation_path>/lib/python3/dist-packages/morse/robots/
# We will replace pr2.py class with our version:
sudo cp pr2.py <morse_installation_path>/lib/python3/dist-packages/morse/robots/
# We will replace sensors.py class with our version:
sudo cp sensors.py <morse_installation_path>/lib/python3/dist-packages/morse/builder/
# We will replace human_posture.py class with our version:
sudo cp human_posture.py <morse_installation_path>/lib/python3/dist-packages/morse/sensors/
# We will replace main.py of blender with our version (a bug fix for overlayed objects):
sudo cp main.py <morse_installation_path>/lib/python3/dist-packages/morse/blender/
# We will replace human_posture.py of blender with our version (a bug fix):
sudo cp human_posture.py <morse_installation_path>/lib/python3/dist-packages/morse/sensors/
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

make sure that your builder script ("hrc_scenario.py" in our case) has this line to import conveyor belt properly:
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
To run morse scene, do the following in a terminal:
```
cd code/
morse import -f hrc_morse # --> this is to import the folder as a morse project. You need to do it once.
```
We need to source the ros workspace for morse too, because morse entities are using ros services as an interface to outside world.
```
source ros_ws/devel/setup.bash # Every time you open a new terminal this is necessary. Or put the command into `.bashrc` file
morse run hrc_morse hrc_morse/hrc_scenario.py # the last argument is showing the location of hrc_scenario.py
```

Now if you run `roscore` in another terminal this will start the morse scene (due to the ros services initialized under morse, we need ros master). You can navigate in the morse scene by pressing `F5` (changes camera view from human to world). and with `W,A,S,D` for direction and `ctrl + mouse` for orientation.

Running ROS project:
Open a new terminal and
```
source ros_ws/devel/setup.bash
roslaunch hrc_ros hrc.launch
```
you can optionally call <hrc_scenario_cam_only.py> for a simplified scenario to use for human activity recognition (a dummy robot with Kinect camera on). 

Every process is run by a rosservice call. Run "rosservice list" to see all available interfaces ! You can simply run each by typing "rosservice call /human/walk_away"
To run the system:
```
rosservice call /task_manager/new_scenario_request
```
This initiates the conveyor belts and a package. Package stops in between human and the robot and two terminals pops out. One terminal is for selecting human states to guide human actions (see human states from 0 to 10) under code/models/human_models (any model has the same human states). For the other terminal, it is robot's decision-making and it operates automatically, just for monitoring purposes. Once a package is stored/failed and fell into the unprocessed container, a scenario ends and you should manually close the two terminals popped out (this will be fixed in the future). A reinvoke of new scenario request will initiate another scenario. Just mind that as the task numbers increases human gets more tired/distracted/thinker leading him to fail to grasp or not attending.

There is a topic where the state of the tasks can be printed: current human model, the state, the action taken, time, the robot state, the reward, action taken ...
```
rostopic echo /task_manager/task_status
```

## MDP, POMDP Models

All the predesigned human and robot models can be found under code/models.
The models are read by the despot packages to generate policies and run them.
You can create your own POMDP models by adjusting the existing .pomdpx files under /models folder. For more information on POMDPs and the .pomdpx file please refer to here: http://bigbird.comp.nus.edu.sg/pmwiki/farm/appl/index.php?n=Main.PomdpXDocumentation.

### Selecting Models
Which model (human and robot types) to run initially can be selected under code/configs/scenario_config.json
Just replace the type fields under the json file with the ones below#
human:
- expertise: "beginner / expert"
- mood:"stubborn / thinker / distracted / tired" # Note that in time human is assigned randomly but more likely with distracted and tired types automatically.
robot: 
- "proactive / reactive" # This will run either POMDP or MDP model respectively.

For more model creation, refer to this shared excel sheet for easily adjust state transition, observation and reward matrices and create your own .pomdpx models:
- https://docs.google.com/spreadsheets/d/1gJoA5ltNewCgFDSOcUGdoqZcWzdyu6Id3xDJE6V_nDg/edit?usp=sharing

### Controlling Human and Robot Models
After you successfully run the a scenario, you will notice that two terminal will open automatically. Those are opened by despot_human and despot_robot packages. They simply execute the human and robot policies generated.
In this version, the robot model runs automatically, the terminal is for you to visualize what reward the robot got, which state (or belief state in POMDP) the robot is in and which action it took. Also you can call the task_status topic for these information. Please see the robot_model designs for MDP and POMDP visaulized below. Taking this connection scheme and the observations mentioned as the base there can be different robot models created by tweaking around with the state transition, observation and reward probabilities.

The terminal opened by despot_human allows you to control the human actions. This is left as it is for easy testing of the expected robot behaviors. Simply follow the human_model drawing given below to input the next state the human is in (an integer from 0 to 9 each referring to a state as given in the drawing). 
The functionality is basically then, you input the next state, the human model selects one action in that state (according to the model design), then the robot observes the action and responds, then you will decide on the next state (in response to the robot action). Although here we have the control over the human, the action selection of the human is still unknown to us, selected by the MDP model. Please ask the author for any further questions.
- Robot Models:
![ngrok](https://gitlab.tubit.tu-berlin.de/app-ras/hrc_industry_ss18/raw/master/doc/robot_models.png)
- Human Models:
![ngrok](https://gitlab.tubit.tu-berlin.de/app-ras/hrc_industry_ss18/raw/master/doc/human_model.png)

## Brief overview of the system

System Architecture drawing of the new version of the project, also showing the nodes to be updated for the student developments.
![ngrok](https://gitlab.tubit.tu-berlin.de/app-ras/hrc_industry_ss18/raw/master/doc/system_architecture_v2.png)

## References
In any use of this code, please let the author know and please cite the articles below:
* O. Can Görür, Benjamin Rosman, Fikret Sivrikaya, and Sahin Albayrak. 2018. Social Cobots: Anticipatory Decision-Making for Collaborative Robots Incorporating Unexpected Human Behaviors. In Proceedings of the 2018 ACM/IEEE International Conference on Human-Robot Interaction (HRI '18). ACM, New York, NY, USA, 398-406. DOI: https://doi.org/10.1145/3171221.3171256


