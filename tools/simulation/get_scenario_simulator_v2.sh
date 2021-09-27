#!/bin/bash

SCRIPT=$(readlink -f $0)
SCRIPT_PATH=`dirname $SCRIPT`
WORKSPACE_ROOT_PATH=$SCRIPT_PATH/../..

if [ -z $ROS_DISTRO ]; then
    echo "ROS_DISTRO environment variable not set. Source ROS2 distro or set it manually."
    exit 1
fi

vcs import $WORKSPACE_ROOT_PATH < $SCRIPT_PATH/scenario.simulator.v2.repos
vcs import $WORKSPACE_ROOT_PATH/src/external/scenario_simulator/external/ < $WORKSPACE_ROOT_PATH/src/external/scenario_simulator/dependency_$ROS_DISTRO.repos

sudo apt-get update
rosdep update
rosdep install --from-paths $WORKSPACE_ROOT_PATH/src --ignore-src -y
