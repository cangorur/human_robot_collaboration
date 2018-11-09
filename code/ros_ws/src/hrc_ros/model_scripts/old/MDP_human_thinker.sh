#! /bin/sh

_mydir="$1"
#change this path below to run different MDP models
$_mydir/../../../despot_MDP_human/build/examples/pomdpx_models/despot_pomdpx -m $_mydir/../../../models/human_models/humanModel_thinker.POMDPx --runs 1
