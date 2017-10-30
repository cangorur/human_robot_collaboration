#! /bin/sh

_mydir=$1
#change this path below to run different pomdp models
${_mydir}/../../../despot_MDP_robot/build/examples/pomdpx_models/despot_pomdpx -m ${_mydir}/../../../models/robot_models/reactive_robot_pomdp.pomdpx --runs 1
