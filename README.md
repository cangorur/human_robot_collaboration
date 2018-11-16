Human-Robot Collaboration on an Assembly Line (Industrial Scenario)
============

by Orhan Can Görür (orhan-can.goeruer@dai-labor.de)

Below are the instructions to succesfully install the current HRC simulation on a conveyor belt (also referring to the other notes) for pick up and place/store scenario. The code also involves autonomous decision-making and actuator models for both human and the robot (pr2). The goal is to test these DM algorithms for the robot in response to human behaviors which can resemble a tired, distracted, thinker, stubborn, beginner and expert human worker types. These situations and randomness in human actions constitute difficult cases for the robot to respond properly and reliably.

For the students, the architecture drawing at the bottom of the page shows the possible improvement points and how they can interfere with the process. For more questions, please contact the mail given above.

---

## Prerequisites
### MORSE

Install Morse 1.3.1-STABLE. It is a forked project. Clone it from here:
```
https://github.com/cangorur/morse.git
```
Install python3-dev package
```
sudo apt install python3-dev
```
Then, create a build folder and cmake and install:
```
mkdir build && cd build
cmake -DBUILD_ROS_SUPPORT=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt ..
sudo make install
```
For the ROS interface of MORSE, follow the instructions here [link](http://www.openrobots.org/morse/doc/1.3/user/installation/mw/ros.html). But first read below and examine the steps in the link*!
- Step 1 and 2 under the link, you have already done.
- NOTE: For Step 3, 4 do the manual installation. That is, they have both apt-get install … OR from source. **Go with “from source” instructions**.
- Step 5 is already with one option.

### ROS

The code is tested with ROS Kinetic on Ubuntu 16.04 machines.
No special ROS packages are needed apart from those which come with a standard installation of ROS.

The only dependency for the installation is the *Boost libraries* (usually cpreinstalled with OS)
For running the python nodes, we have extra dependencies:
* numpy (sudo apt install python3-numpy, also install for python2: ROS is running that) --> you can install with pip,
* scipy (sudo apt install python3-scipy, also install for python2: ROS is running that) --> you can install with pip,

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

## Running
### Running Simulation With MORSE
To run morse scene, do the following in a terminal:
```
cd code/
morse import -f hrc_morse # --> this is to import the folder as a morse project. You need to do it once.
```
We need to source the ros workspace for morse too, because morse entities are using ros services as an interface to outside world.
```
source ros_ws/devel/setup.bash # Every time you open a new terminal this is necessary. Or put the command into .bashrc file
```
Finally, we run the morse project
```
morse run hrc_morse hrc_morse/hrc_scenario.py # the last argument is showing the location of hrc_scenario.py
```

Now if you run `roscore` in another terminal this will start the morse scene (due to the ros services initialized under morse, we need ros master). You can navigate in the morse scene by pressing `F5` (changes camera view from human to world). and with `W,A,S,D` for direction and `ctrl + mouse` for orientation.

- Hints for adding a conveyor belt:
Note that if you develop your own builder script (e.g. "hrc_scenario.py" in our case), make sure that it has this line to import conveyor belt properly:
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

### Running Simulation Without MORSE (EXPERIMENTAL)
For model testing or faster data recording, we created an option to not to run MORSE to simulate. The human agent (action executor) is also created as a ROS node so that when run the ROS agents send action commands to this node (actions are simulated as sleep functions) and it replies with the observations back to the ROS agents. The simulation this way is executed without running MORSE environment.
To run:
```
source ros_ws/devel/setup.bash
rosrun hrc_ros human_sim_noMorse.py
```
Here, make sure `human_sim_noMorse.py` is executable and `roscore` is running.

### Running ROS Project
After MORSE project is running, open a new terminal and
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
This initiates the conveyor belts and a package. Package stops in between human and the robot and two terminals pops out. One terminal is for selecting human states to guide human actions (see human states from 0 to 10) under code/models/human_models (any model has the same human states). For the other terminal, it is robot's decision-making and it operates automatically, just for monitoring purposes. Once a package is stored/failed and fell into the unprocessed container, a scenario ends and you should manually close the two terminals popped out (this will be fixed in the future). A reinvoke of new scenario request will initiate another scenario.

There is a topic where the state of the tasks can be printed: current human model, the state, the action taken, time, the robot state, the reward, action taken ...
```
rostopic echo /task_manager/task_status
```
### Configuration of the Project
This project provides flexibility in selecting different human and robot models for the interaction scenario. The configuration is done under `code/configs/scenario_config.json`.
For now, in a repeated interaction mode (Evaluation mode), since we use only one package the package pool and the package configurations are not affective.
Most important features to update are, `human`, `robot`, `operation_modes` and `evaluation_models`.
- `human` allows to select one particular human model to run, when `useEvaluator` is NOT activated. For each human type we created one model (for the latest versions please see `/code/models/human_models/Evaluate`). The type consists of three characteristics. `expertise` can be `beginner, expert` (leave empty for `distracted` type), `mood` can be `tired, nontired, distracted`, `collaborativeness` can be `collaborative, noncollaborative`.
- `robot` similar to the human, this allows to select one particular robot model to run, when `useEvaluator`, `useCMAB` and `useBPR` are all deactive!. Please refer to `/code/models/robot_models/Evaluate` for possible robot policies. To run one of them, simply put its name to `AItype` variable (*without the .pomdx file extension*) and leave the rest empty under the config file. Under TaskManager.cpp, these strings are simply added to create one robot type string. So, make sure you levae the rest empty.
- `operation_modes` is to define if the scenario run is for training (data collection), a run with some policy selectors and to use dynamic transitions on human models for better performance test of the policy selector.
- `useEvaluator`: if this is active, then the `TaskManager.cpp` calls the `policy_evaluator` node to retrieve one human and one robot model to interact several times (defined under `interactionNumber`). The purpose is to run every single human model with robot models to collect performance information (which policy is the best for which human) and to collect human observation data for type estimation algorithms of the robot. The models to be tested under this mode are provided under `evaluation_models` for human and the robot (currently all existing models are put there).
- `useCMAB` and `useBPR`: These are two implemented policy selection algorithms (Contextual Multi-Arm Bandit and Bayesian Policy Reuse) to select the best policy for a robot interacting with a certain (unknown) human type. This is for the real time test run of the robot and the algorithms use the trained models after `useEvaluator` mode. The model file locations are given under `trainingSetForPolicySelect`.
- For the student version of the code, `useCMAB` and `useBPR` modes are not implemented to promote students to develop their own policy generation (RL) / policy selection approaches. These may be replaced by the students' own solutions. Please note that `useCMAB` in the original version is not stable; however, `useBPR` works fine. But it is not a RL algorithm.
- *The procedure for Training Model creation*: After the evaluation mode has terminated (all combinations are tested), please ctrl+c (terminate) the ros window and you should see a `.bag` file created under the `Results` folder. This bag file holds the recorded task status topic: `/task_manager/task_status`. Examine the `README` file under the results folder for further possible analysis.
  - First step should be to convert bag to a csv file. run
```
python bag_to_csv.py <your_bag_file_here>
```
  - `raw_data.csv` excel sheet holds all the information.
  - In the original version, we have training set creations for BPR implementation. However, for the student version the students should come up with their own training strategies according to their RL / human type estimation implementations (e.g. any classification algorithm, RL algos etc).
  - `raw_data.csv` has the informed status of each of the agents. `human` agent outputs his current action, state etc, `observation` agent informs about the human and task state observables to be used under robot models, `robot` agent informs about the momentary action selected, belief state and the immediate and accumulated discounted rewards gathered.
- `useTransitionFunction` is the last configuration feature that is not stable and under improvement. The intention is to have human models change their state transitions on the fly, in response to the time, task and the robot actions. E.g. a human getting more annoyed with the robot is reflected by warning the robot more frequently. Such a dynamic transition function is to be implemented under `human_mc_sampler` agent. The function is left there for improvement, named `modifyPOMDPx`. This agent is responsible for sampling a human model's next state using Monte Carlo Markov Chain sampling at every step.

### Hints for Flexible Runs:
The project allows to reset a task, start from any task number, start from any different scenario (for the `useEvaluator` mode, a scenario means a combination of human-robot model interacting) and control human and robot actions from the terminal (for testing and debugging purposes).
- To reset a task (this overwrites the `task_number` variable under `TaskManager` and initiates a new scenario), simply run:
```
rosservice call /task_manager/reset_task
```
- During the operation or before we start new scenario request, we can set the task number and scenario count (as mentioned). An example would be, scenario_count 10 refers to the second human vs robot model combination under evaluation models given that interactionNumber for each combination is set to be 10 (i.e. the count starts from 0). For this we should first activate a parameter to be able to interfere:
```
rosparam set /training_reset True
rosparam set /task_count <integer number>
rosparam set /scenario_count <integer number> # be careful that the scenario counts depend on the interactionNumber and the evaluation_models provided under the config files.
NOTE THAT: This update in its experimental phase and task count and scenario count need to be consisting with each other in terms of the interactionNumber defined for each scenario.
```
- Human and robot actions can be controlled manually. These actions are implemented under MORSE; therefore, the ros services are provided from there. See the list of human and robot actions by running rosservice list when MORSE project is on. Some possible action calls (the most useful one would be /reset to put human and robot back to their original positions and canceling previous actions):
```
rosservice call /human/reset # replace reset with look_around OR warn_robot OR grasp OR walk_away OR sit_down OR stand_up (the last two actions are not included under human models)

rosservice call /robot/reset # replace reset with grasp OR point_to_obj OR cancel_action (cancel whatever it is doing) OR planning_for_motion (this is implemented as a timer blocking robot doing anything else to simulate planning for kinematics for any action)
```
## MDP, POMDP Models

All the predesigned human and robot models can be found under code/models.
The models are read by the despot packages to generate policies and run them.
You can create your own POMDP models by adjusting the existing .pomdpx files under /models folder. For more information on POMDPs and the .pomdpx file please refer to here: http://bigbird.comp.nus.edu.sg/pmwiki/farm/appl/index.php?n=Main.PomdpXDocumentation.

### Selecting Models
Which model (human and robot types) to run initially can be selected under `code/configs/scenario_config.json`
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
![ngrok](https://gitlab.tubit.tu-berlin.de/app-ras/hrc_industry/raw/master/doc/robot_models.png)
- Human Models:
![ngrok](https://gitlab.tubit.tu-berlin.de/app-ras/hrc_industry/raw/master/doc/human_model_v2.png)

## Brief overview of the system

System Architecture drawing of the new version of the project, also showing the nodes to be updated for the student developments.
![ngrok](https://gitlab.tubit.tu-berlin.de/app-ras/hrc_industry/raw/master/doc/system_architecture_v2.png)

## References
In any use of this code, please let the author know and please cite the articles below:
* O. Can Görür, Benjamin Rosman, Fikret Sivrikaya, and Sahin Albayrak. 2018. Social Cobots: Anticipatory Decision-Making for Collaborative Robots Incorporating Unexpected Human Behaviors. In Proceedings of the 2018 ACM/IEEE International Conference on Human-Robot Interaction (HRI '18). ACM, New York, NY, USA, 398-406. DOI: https://doi.org/10.1145/3171221.3171256

## Running the interaction experiment 

In the real world interaction experiment example, some nodes are different and some services slightly changed. 

The nodes that changed: 
- task_manager is now task_manager_IE
- observation_manager is now observation_manager_IE 
- TODO - others to follow 

How to start the system:
``` 
rosservice call /task_manager_IE/new_scenario_request
```

Additional: 
- for now *useEvaluator* has to be set to *false* in the *scenario_config.json*