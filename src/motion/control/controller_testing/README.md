Controller Testing- Usage and Configuration
=============
This package contains launch files, param and scripts for 
running simulted controller tests with dynamic vehicle model, and generate plots



# Params

* MPC controller config 
`mpc_controller_node/param/defaults.yaml`
* Test and simulation config
`controller_testing/param/defaults.param.yaml`

# Usage

* For faster than realtime use `real_time_sim:=False`

```
ros2 launch controller_testing controller_testing_node.launch.py real_time_sim:=False with_rviz:=False
```

* For realtime use with rviz
```
ros2 launch controller_testing controller_testing_node.launch.py real_time_sim:=True with_rviz:=True
```
