# despot-online-executor

## Overview

[Copyright &copy; 2014-2017 by National University of Singapore](http://motion.comp.nus.edu.sg/).

This package is an upgrade to DESPOT<sup>1</sup> package (https://github.com/AdaCompNUS/despot) for our specific use to execute the generated MDP policies in real-time. The package serves as a decision-making tool for any autonomous systems allowing the communication through a websocket. 

In principal:
* Once executed, the package solves for a given **POMDPX** file specified, then runs the policy step-by-step waiting for input from the environment. 
* Input is in the format of **State** to the MDP model that leads the model to calculate its belief state and output an **action** decision and the previous **reward** the robot has received. 

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
$ ./<path-to-despot-executor>/build/examples/pomdpx_models/despot_pomdpx -m <path-to-despot-executor>/examples/pomdpx_models/data/humanModel_v2.POMDPx --runs 1
```
The model details can be found under here: https://docs.google.com/spreadsheets/d/1jDDyNXrNnYsDy5L82CDVipNZQwEvev44tx2FqHkm2wE/edit?usp=sharing. The observations and the states are numbered described. Please contact me in case of your model specific questions.

## Configuration and Usage

First of all, as DESPOT follows **POMDPX** file format it is recommended to get familiar with it from here: (http://bigbird.comp.nus.edu.sg/pmwiki/farm/appl/index.php?n=Main.PomdpXDocumentation)

*Once the model is prepared, DESPOT numbers the states, the observations and the actions starting from "0" in the order they are provided in the pomdpx file.*

**Input Channel:**
Once it is running, the terminal prompts and asks for the current state information (in MDP systems state information is the input to the policy). The state information is in integer numbers as mentioned above. 


**Output Channel:**
Each time a state is provided, the package outputs first the state information and then the action decision along with the belief state. The state info output is actually the real state input manually. It is just forwarded thru the sockets to let the SW system know.

The output channel from the package is trough a websocket with port number: **9090**. The package connects to a server on that part as a client. This is to ensure the synchronization of the package with the other packages and architectures. So, to listen the outputs a servier should be initiated on the port **9090**.

The output message format:
```
msg_step_results: "action_decision_number" + "," + "state_str";
```
The message always have the same format, but never informs all those info above in the same message. Once a manual input state received only this state is being forwarded where the *action_decision_number* is "-1". Then, afterwards the action generated is sent this time *state_str* informs the belief state of MDP.

**Terminating:**
In complex systems it is very hard to define a terminal state through the model design directly. For our own use, we have manually defined terminal states according to our model. Once this state is provided as a real state in any iteration, the package terminates. Currently one should manually input in the source code what those terminal states are (still a *TODO*). According to our *proactive_robot_pomdp.pomdpx* model, the terminal states are provided as number "8" or "9" (success and failure) under:

$ cd <path-to-despot-executor>/src/evaluator.cpp  --> [Line 249](https://gitlab.tubit.tu-berlin.de/app-ras/hrc_industry_ss18/blob/initial_commit_version/code/despot_MDP_human/src/evaluator.cpp#L249) (provide your own terminal states here)

## References

[1] Nan Ye, Adhiraj Somani, David Hsu, and Wee Sun Lee. 2017. DESPOT: Online POMDP planning with regularization. Journal of Artificial Intelligence Research 58 (2017), 231–266.

[2] O. Can Görür, Benjamin Rosman, Fikret Sivrikaya, and Sahin Albayrak. 2018. Social Cobots: Anticipatory Decision-Making for Collaborative Robots Incorporating Unexpected Human Behaviors. In HRI ’18: 2018 ACM/IEEE International Conference on Human-Robot Interaction, March 5–8, 2018, Chicago, IL, USA. ACM, New York, NY, USA, 9 pages. https://doi.org/10.1145/3171221.3171256

[Copyright &copy; 2014-2017 by National University of Singapore](http://motion.comp.nus.edu.sg/).
