#! /bin/sh

_mydir="$1"
#change this path below to run different MDP models
$_mydir/../../../despot_MDP_human/build/examples/pomdpx_models/despot_pomdpx -m $_mydir/../../../models/human_models/humanModel_stubborn.POMDPx --runs 1

#$_mydir/../../../../../../Programs/mdp-solve-run/run_terminal/build/examples/pomdpx_models/despot_pomdpx -m $_mydir/../../../models/human_models/humanModel_beginner.POMDPx --runs 1
