vehicle_constants_manager_nodes {#vehicle-constants-manager-nodes-package-design}
===========

This is the design document for the `vehicle_constants_manager_nodes` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This node is responsible for loading, generating derived parameters and
publishing vehicle specific constants as ROS2 Parameters.

This node was implemented to provide various vehicle specific constants to other
nodes from a centralized place.

In order for other nodes to access these parameters, it is recommended to use
`vehicle_constants_manager` package. You can refer to
[vehicle_constants_manager-design](@ref vehicle-constants-manager-package-design)
file.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

## Assumptions / Known limits
<!-- Required -->

This library assumes the vehicle is defined with Ackermann steering geometry.

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

A file that contains fields in the example
`param/params_lexus_rx_hybrid_2016.yaml` should be provided to the node.

```bash
cd ~/adehome/Autoware.Auto
source install/setup.bash

# either with a launch file
ros2 launch vehicle_constants_manager_nodes vehicle_constants_manager_node.launch.py

# or ros2 run
ros2 run vehicle_constants_manager_nodes vehicle_constants_manager_node_exe \
--ros-args --params-file \
~/adehome/Autoware.Auto/src/common/vehicle_constants_manager_nodes/param/params_lexus_rx_hybrid_2016.yaml

# or provide parameters
ros2 run vehicle_constants_manager_nodes vehicle_constants_manager_node_exe \
--ros-args \
-p wheel_radius:=0.37 \
-p wheel_width:=0.27 \
-p wheel_base:=2.734 \
-p wheel_tread:=1.571 \
-p overhang_front:=1.033 \
-p overhang_rear:=1.021 \
-p overhang_left:=0.3135 \
-p overhang_right:=0.3135 \
-p vehicle_height:=1.662 \
-p cg_to_rear:=1.367 \
-p tire_cornering_stiffness_front:=0.1 \
-p tire_cornering_stiffness_rear:=0.1 \
-p mass_vehicle:=2120.0 \
-p inertia_yaw_kg_m2:=12.0
```

It constructs a `vehicle_constants_manager::VehicleConstants` object.

During the construction of the object, the constructor performs sanity checks.

It publishes the primary and derived parameters using ROS2 parameter server.

If another node wants to reach these parameters, it is recommended to follow the
steps in 
[vehicle_constants_manager-design.md / Inputs / Outputs / API](@ref vehicle-constants-manager-package-design-inputs).

## Inner-workings / Algorithms
<!-- If applicable -->
Not Available.

## Error detection and handling
<!-- Required -->
If not all parameters are provided or construction of the 
`vehicle_constants_manager::VehicleConstants` object fails due to sanity checks,
`std::runtime_error` will be thrown with the respective error message.

# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->
To Be Determined.


# References / External links
<!-- Optional -->
Not Available.


# Future extensions / Unimplemented parts
<!-- Optional -->
Not Available.


# Related issues
<!-- Required -->
https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1294
