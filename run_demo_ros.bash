#!/bin/bash

set -e

source install/setup.bash

ros2 launch autoware_auto_launch autoware_auto_visualization.launch.py & ros2 launch autoware_demos avp_sim.launch.py
