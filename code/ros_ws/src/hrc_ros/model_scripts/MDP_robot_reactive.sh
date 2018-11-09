#! /bin/sh

_mydir=$1
robot_model=$2
#change this path below to run different mdp models
${_mydir}/../../../despot_MDP_robot/build/examples/pomdpx_models/despot_pomdpx -m ${_mydir}/../../../models/robot_models/${robot_model} --runs 1
