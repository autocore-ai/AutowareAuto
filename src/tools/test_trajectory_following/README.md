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
ros2 launch test_trajectory_following simple_trajectory_following.launch.py sim_type:=dynamics  # Default
or
ros2 launch test_trajectory_following simple_trajectory_following.launch.py sim_type:=kinematics
or
ros2 launch test_trajectory_following simple_trajectory_following.launch.py sim_type:=lgsvl
```

## 3. Record Replay

```
ros2 launch test_trajectory_following trajectory_recording.launch.py
```

* on Joystick press `start` button to `record`
* drive vehicle manually in simulator/real world (not using the joystick)
* press `back` button to `stop recording`
* reset position of vehicle manually before replaying (may restart simulation)
* press `logo/home` button to start `replay`
  * If `with_mpc:=True` Let MPC drive or else `with_mpc:=False` drive vehicle manually in LGSVL
  * If `with_obstacle:=True` trajectory replay should handle obstacles from simulation (perception stack also launched).
* press `back` button to `stop replaying`

## 4. Trajectory spoofer
Trajectory spoofer is very similar to `simple_trajectory`,
but this can additionally generate more complicated shapes.

Edit the config to publish a circle instead of a straight line.


* Trajectory spoofer config
```test_trajectory_following/param/trajectory_spoofer.param.yaml```

### Test with Lgsvl
```
ros2 launch test_trajectory_following trajectory_spoofer_mpc_control.launch.py sim_type:=lgsvl
```
### Test with headerless dynamics simulator
```
ros2 launch test_trajectory_following trajectory_spoofer_mpc_control.launch.py sim_type:=dynamics
````
### Test with headerless dynamics simulator, faster than realtime mode
```
ros2 launch test_trajectory_following trajectory_spoofer_mpc_control.launch.py sim_type:=dynamics real_time_sim:=False with_rviz:=False
```
### Test with headerless kinematics simulator
```
ros2 launch test_trajectory_following trajectory_spoofer_mpc_control.launch.py sim_type:=kinematics
```

## Toubleshooting
Test kinematics_sim using joystick
```
ros2 launch test_trajectory_following test_joystick_vehicle_kinematics_sim.launch.py
```
