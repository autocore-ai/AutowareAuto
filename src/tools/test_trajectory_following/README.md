Test Trajectory Following - Usage and Configuration
=============
This package contains launch files, param and scripts for 
easier integration testing of different modules needed for trajectory following functionality.

For exmaple this package can be used to test `controller` and `simulator` with dummy/spoofed trajectory.

# Usage

## 1. Common 

* Start LGSVL and ros_bridge

* MPC controller config 
`test_trajectory_following/param/mpc_controller.param.yaml`


## 2. Simple trajectory following
* Trajectory generation config 
```test_trajectory_following/param/simple_trajectory.param.yaml```

```
ros2 launch test_trajectory_following simple_trajectory_following.launch.py
```

## 3. Record Replay

```
ros2 launch test_trajectory_following trajectory_recording.launch.py
```
* on Joystick press `start` button to `record`
* drive vechile manully in simulator/real world (not using joystick)
* press `back` button to `stop recording`
* press `logo/home` bnutton to start `replay`
* press `back` button to `stop replaying`

## 4. Trajectory spoofer 
Trajectory spoofer is very similar to `simple_trajectory`, 
but this can additionally generate more complicated shapes.

Edit the config to publish a circle instead of a straight line.


* Trajectory spoofer config 
```test_trajectory_following/param/trajectory_spoofer.param.yaml```

```
ros2 launch test_trajectory_following  trajectory_spoofer_mpc_control_lgsvl.launch.py
```