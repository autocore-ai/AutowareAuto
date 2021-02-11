Trajectory Following {#trajectory-following-howto}
============

@tableofcontents

# Simple trajectory following in LGSVL

The [test_trajectory_following](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/tools/test_trajectory_following) package contains launch files, param and scripts for easier integration testing of different modules needed for trajectory following functionality.

For example this package can be used for simulator testing with a dummy/spoofed trajectory.
To do so, follow the instructions below:

* Start [LGSVL](lgsvl.html)
* Launch the trajectory test node:
```console
ros2 launch test_trajectory_following simple_trajectory_following.launch.py sim_type:=lgsvl
```

The launch file start spoofing a trajectory and bring up an rviz visualization.
By default green is the trajectory from planner/record replay, whatever is being sent to the controller, and blue is the internal representation of the mpc controller, which may be interpolated, smoothed, and predicted.

![Autoware.Auto trajectory following](trajectory-following-small.jpg)
