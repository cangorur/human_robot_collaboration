# DESPOT

## How the simulation/evaluation works: the tracking of the source code:
First the story: It initialize the model as DSPOMDP model. Runs the evaluator and initialize the model belief. Then in a for loop (under simleTUI) we RunStep(). This will first seearches for an action (agent model searches), then executes this action (simulator executes), then the simulator calculates the new state, and calculates an observation. Finally, the agent (despot.cpp class, and model object under belief.cpp) takes this observation given the previous action it executed and calculates the new belief (the new estimated state).
Currently, all the belief states estimated are the same with the states calculated by the simulator.?
* main --> TUI.run(), i.e., SimpleTUI.cpp --> Run()
* DSPOMDP model = InitializeModel()
* Evaluator simulator = InitializeEvaluator() , i.e. evaluator.cpp POMDPEvaluator class
* Solver solver, i.e. solver initialzed under despot.cpp as "despot" class 
* simpleTUI --> RunEvaluator(model, simulator etc.)
* simulator --> InitRound(), i.e. evaluator.cpp InitRound --> this calculates initial belief model-->initialBelief() , where model is under pomdpx.cpp
* simulator --> RunStep(), i.e. evaluator.cpp RunStep()
* under RunStep() : solver-->Search().action ->This calls Search() function under despot.cpp
* ExecuteAction() --> under evaluator.cpp POMDPEvaluator class has this function
* under ExecuteAction() --> model --> Step() , i.e. this model is under pomdpx.cpp and the Step() function is located there.
* under model-->Step(), parser --> GetNextState() + GetReward() + GetObservation() sequentially. THIS SEQUENCE IS VERY IMPORTANT. all under parser.cpp
* Then, under evaluator.cpp RunStep() function: the state, action, reward, observation are all printed out
* finally, solver --> Update(action, obs) . This update function under despot.cpp, which then triggers the model Update() function under belief.cpp: update belief given the action, obs, and current state.

## Overview
Approximate POMDP Planning Online (APPL Online) Toolkit

This software package is a C++ implementation of the DESPOT algorithm<sup>1</sup>.

[1] [**DESPOT: Online POMDP Planning with Regularization**](https://www.jair.org/media/5328/live-5328-9753-jair.ps). *Nan Ye, Adhiraj Somani, David Hsu and Wee Sun Lee*. 
This implementation extends our [NIPS 2013 paper](http://bigbird.comp.nus.edu.sg/pmwiki/farm/motion/uploads/Site/nips13.pdf) with an improved search algorithm, analysis and more empirical results.

[Copyright &copy; 2014-2017 by National University of Singapore](http://motion.comp.nus.edu.sg/).

## Requirements

Tested Operating Systems:

<!--| Linux 14.04| OS X (10.1)  | Windows  |
|:------------- |:-------------:|: -----:|
|[![Build Status](https://semaphoreapp.com/api/v1/projects/d4cca506-99be-44d2-b19e-176f36ec8cf1/128505/shields_badge.svg)](https://semaphoreapp.com/boennemann/badges)| [![Build Status](https://semaphoreapp.com/api/v1/projects/d4cca506-99be-44d2-b19e-176f36ec8cf1/128505/shields_badge.svg)](https://semaphoreapp.com/boennemann/badges) | Not Supported |-->

| Linux       | OS X
| :-------------: |:-------------:|
|[![Build Status](https://semaphoreapp.com/api/v1/projects/d4cca506-99be-44d2-b19e-176f36ec8cf1/128505/shields_badge.svg)](https://semaphoreapp.com/boennemann/badges)      | [![Build Status](https://semaphoreapp.com/api/v1/projects/d4cca506-99be-44d2-b19e-176f36ec8cf1/128505/shields_badge.svg)](https://semaphoreapp.com/boennemann/badges) 

Tested Compilers: gcc | g++ 4.2.1 or above

Tested Hardware: Intel Core i7 CPU, 2.0 GB RAM

Other Dependencies: (Optional) [CMake (2.8+)](https://cmake.org/install/)

## Installation

Clone and compile:
```bash
$ git clone https://github.com/AdaCompNUS/despot.git
$ cd despot
$ make
```

## Examples

DESPOT can be used to solve a POMDP specified in the **POMDPX** format or a POMDP
specified in **C++** according to the API. We illustrate this on the [Tiger](http://people.csail.mit.edu/lpk/papers/aij98-pomdp.pdf) problem.

1.To run Tiger specified in [C++](doc/cpp_model_doc), compile and run: 
```bash
$ cd despot/examples/cpp_models/tiger
$ make
$ ./tiger --runs 2
```

This command computes and simulates DESPOT's policy for `N = 2` runs and reports the
performance for the tiger problem specified in C++. See [doc/Usage.txt](doc/Usage.txt) for more options.

2.To run Tiger specified in [POMDPX format](http://bigbird.comp.nus.edu.sg/pmwiki/farm/appl/index.php?n=Main.PomdpXDocumentation.), compile and run:

```bash
$ cd despot/examples/pomdpx_models
$ make
$ ./pomdpx -m ./data/Tiger.pomdpx --runs 2 
```

This command computes and simulates DESPOT's policy for `N = 2` runs and reports the
performance for the tiger problem specified in POMDPX format. See [doc/Usage.txt](doc/Usage.txt) for 
more options.


## Integration

To install DESPOT libraries and header files for external usage, use the [CMakeLists.txt](CMakeLists.txt) provided:
```bash
$ cd despot
$ mkdir build; cd build
$ cmake ../
$ make
$ sudo make install
```

To integrate DESPOT into your project, add this to your `CMakeLists.txt` file:

```CMake
find_package(Despot CONFIG REQUIRED)

add_executable("YOUR_PROJECT_NAME"
  <src_files>
)

target_link_libraries("YOUR_PROJECT_NAME"
  despot
)

```

## Documentation

Documentation can be found in the "[doc](doc/)" directory. 

For a description of our example domains and more POMDP problems see [the POMDP page](http://www.pomdp.org/examples/).


## Package Contents

```
Makefile                  Makefile for compiling the solver library
README.md                 Overview
include                   Header files
src/core                  Core data structures for the solvers
src/solvers               Solvers, including despot, pomcp and aems
src/pomdpx                Pomdpx and its parser
src/util                  Math and logging utilities
src/ippc                  Interface for International Probabilistic Planning Competition
license                   Licenses and attributions
examples/cpp_models       POMDP models implemented in C++
examples/pomdpx_models    POMDP models implemented in pomdpx
doc/pomdpx_model_doc      Documentation for POMDPX file format
doc/cpp_model_doc         Documentation for implementing POMDP models in C++
doc/Usage.txt             Explanation of command-line options
doc/nips2013.txt          Instruction to obtain results in [1]
```

## Acknowledgements

Pocman implementation and memorypool.h in the package are based on David
Silver's [POMCP code](http://www0.cs.ucl.ac.uk/staff/D.Silver/web/Applications.html)

## Bugs and Suggestions
Please use the issue tracker.

## Release Notes
2015/09/28 Initial release.

2017/03/07 Public release. Revised documentation.

