# despot-online-executor

## Overview

[Copyright &copy; 2014-2017 by National University of Singapore](http://motion.comp.nus.edu.sg/).

This package is an upgrade to DESPOT<sup>1</sup> package (https://github.com/AdaCompNUS/despot) for our specific use to execute the generated policies in real-time. The package serves as a decision-making tool for any autonomous systems allowing the communication through a websocket. 

In principal:
* Once executed, the package solves for a given **POMDPX** file specified, then runs the policy step-by-step waiting for input from the environment. 
* First input is in the format of **Observation** to the POMDP model that leads the model to calculate its belief state and output an **action** decision. 
* Then, the package expects for a real **State** information to output a **reward** the robot has received. 
Please note that the model has its own belief update and this real state information is just for the robot to calculate its reward from the last action decision. Inputs and outputs are communicated through the websockets.

For further information (including documentation) please refer to the readme file of the original DESPOT repository: https://github.com/AdaCompNUS/despot

## Installation

```bash
$ mkdir build
$ cd build && cmake .. && make
```

## Running

To run a model (after installing):

```bash
$ cd <path-to-despot-executor>/build/despot/examples/pomdpx_models
$ ./despot_pomdpx -m <path-to-your-pomdpx-file> --runs 1 
```

There is an example pomdpx files provided with the package. This robot model is used for anticipatory Human-Robot Collaboration in an assembly line task <sup>2</sup>:
```bash
$ ./<path-to-despot-executor>/build/examples/pomdpx_models/despot_pomdpx -m <path-to-despot-executor>/examples/pomdpx_models/data/proactive_robot_pomdp.pomdpx --runs 1
```
The model details can be found under here: https://docs.google.com/spreadsheets/d/18NoPhiF1fviwedasoB94-7EnJnLu-djLcfcmjv1YyZw/edit?usp=sharing. The observations and the states are numbered described. Please contact me in case of your model specific questions.

## Configuration and Usage

First of all, as DESPOT follows **POMDPX** file format it is recommended to get familiar with it from here: (http://bigbird.comp.nus.edu.sg/pmwiki/farm/appl/index.php?n=Main.PomdpXDocumentation)

*Once the model is prepared, DESPOT numbers the states, the observations and the actions starting from "0" in the order they are provided in the pomdpx file.*

**Input Channel:**
The input channel to the package is through a websocket with port number: **7070**. The package starts a server on that port.
It expects first the observation then the real state info in separate consecutive messages. Send the messages below to successfully communicate:
```
str_msg_Observation = "<observation_number>" + "," + "-1";
str_msg_RealState = "-1" + "," + "<real_state_number>";
```
Basically, the message has a certain format <"str_observation,str_state">. When any of them is NOT "-1" system understands that information is provided.

**Output Channel:**
Each time an observation is provided, the model prints out the belief distribution and generates an action decision. Afterwards it expects for a real state information and calculates and outputs rewards.

The output channel from the package is trough a websocket with port number: **8080**. The package connects to a servier on that part as a client. This is to ensure the synchronization of the package with the other packages and architectures. So, to listen the outputs a servier should be initiated on the port **8080**.

The output message format:
```
msg_step_results: "action_decision_number" + "," + "current_belief_state_str" + "," + "immediate_reward" + "," + "immediate_disc_reward";
```
The message always have the same format, but never informs all those info above in the same message. Once an obs received only the action and belief are provided rest being "-1". Whereas after the real state info is received the action will be "-1" this time prodiving immediate rewards.

**Example:**

Please see the example client script [here](https://github.com/cangorur/despot-online-executor/blob/master/test_pomdp_client.py).

After running the despot package, run this client script to start the communication. It only prompts for inputing observation and real state information, prepares the msg format and sends it to the despot. 
```bash
$ python test_pomdp_client.py
```

**Terminating:**
In complex systems it is very hard to define a terminal state through the model design directly. For our own use, we have manually defined terminal states according to our model. Once this state is provided as a real state in any iteration, the package terminates. Currently one should manually input in the source code what those terminal states are (still a *TODO*). According to our *proactive_robot_pomdp.pomdpx* model, the terminal states are provided as number "8" or "9" (success and failure) under:

$ cd <path-to-despot-executor>/src/evaluator.cpp  --> [Line 229](https://github.com/cangorur/despot-online-executor/blob/cb5f4d86825a1f67c317cb47c12d9bbb24747636/src/evaluator.cpp#L229) (provide your own terminal states here)

## References

[1] Nan Ye, Adhiraj Somani, David Hsu, and Wee Sun Lee. 2017. DESPOT: Online POMDP planning with regularization. Journal of Artificial Intelligence Research 58 (2017), 231–266.

[2] O. Can Görür, Benjamin Rosman, Fikret Sivrikaya, and Sahin Albayrak. 2018. Social Cobots: Anticipatory Decision-Making for Collaborative Robots Incorporating Unexpected Human Behaviors. In HRI ’18: 2018 ACM/IEEE International Conference on Human-Robot Interaction, March 5–8, 2018, Chicago, IL, USA. ACM, New York, NY, USA, 9 pages. https://doi.org/10.1145/3171221.3171256

[Copyright &copy; 2014-2017 by National University of Singapore](http://motion.comp.nus.edu.sg/).
