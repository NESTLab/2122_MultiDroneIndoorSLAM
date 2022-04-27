#!/bin/bash

# run an argos environment with a ros launchfile
ros_name=$2
exp_name=$1
ARGOS_EXP=${PWD}/argos_worlds/${exp_name}.argos
argos3 -c ${ARGOS_EXP} &
roslaunch ${PWD}/roslaunch_files/${ros_name}.launch
