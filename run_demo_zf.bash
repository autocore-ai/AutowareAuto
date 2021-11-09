#!/bin/bash

set -e

source install/setup.bash

ros2 launch autoware_auto_launch autoware_auto_visualization.launch.py & runtime --graph-file autoware_demo.yaml --runtime local
