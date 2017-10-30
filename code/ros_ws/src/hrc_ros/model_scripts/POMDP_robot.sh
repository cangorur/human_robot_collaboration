#! /bin/sh

_mydir=$1
#change this path below to run different pomdp models
${_mydir}/../../../despot_POMDP_robot/build/examples/pomdpx_models/despot_pomdpx -m ${_mydir}/../../../models/robot_models/pomdp_FINAL.pomdpx --runs 1
