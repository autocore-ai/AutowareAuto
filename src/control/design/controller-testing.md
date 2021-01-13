Controller Testing {#controller-testing}
=============
This package contains launch files, param and scripts for
running simulted controller tests with dynamic vehicle model, and generate plots



# Params

* MPC controller config
`mpc_controller_nodes/param/defaults.yaml`
* Test and simulation config
`controller_testing/param/defaults.param.yaml`

# Usage
- MPC
```
ros2 launch controller_testing controller_testing_node_mpc.launch.py real_time_sim:=True with_rviz:=True
```

-  Pure Pursuit
```
ros2 launch controller_testing controller_testing_node_pure_pursuit.launch.py real_time_sim:=True with_rviz:=True
```

* For faster than realtime use, change arguments into `real_time_sim:=False` and `with_rviz=False`.
