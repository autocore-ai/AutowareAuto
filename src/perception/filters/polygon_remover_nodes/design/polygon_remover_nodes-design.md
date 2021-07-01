polygon_remover_nodes {#polygon-remover-nodes-package-design}
===========

This is the design document for the `polygon_remover_nodes` package.

# Purpose / Use cases

<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This node allows user to remove points that belong to the ego-vehicle from a
PointCloud2.

It wraps around
the [`polygon_remover`](/src/perception/filters/polygon_remover/design/polygon_remover-design.md)
library and exposes it as a ROS2 node.

<img src="/src/perception/filters/polygon_remover/design/polygon_remover_perspective.png" width="25%" height="25%">
<img src="/src/perception/filters/polygon_remover/design/polygon_remover_top_down.png" width="25%" height="25%">

- Red Points: Input Point Cloud
- Yellow Points: Output Point Cloud

# Design

<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
This node has working modes that differentiates how polygon is fed to the node:

- Static vertices as parameters
- Shape Subscriber

## Assumptions / Known limits

<!-- Required -->
Explained in detail here:
[`polygon_remover`](/src/perception/filters/polygon_remover/design/polygon_remover-design.md)

## Inputs / Outputs / API

<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

Subscribers:
- `points_xyzi` `(sensor_msgs::msg::PointCloud2)`

Publishers:
- `cloud_polygon_removed` `(sensor_msgs::msg::PointCloud2)`
  - Publishes point cloud where polygon area is removed.
- `marker_polygon_remover` `(visualization_msgs::msg::Marker)`
  - Will publish the marker of the polygon if `will_visualize` is set to `True`

Setting Parameters:

```bash
cd ~/adehome/Autoware.Auto
source install/setup.bash

# either with a launch file
ros2 launch polygon_remover_nodes polygon_remover_nodes.launch.py

# or ros2 run
ros2 run polygon_remover_nodes polygon_remover_node_exe --ros-args --params-file ~/AutowareAuto/src/perception/filters/polygon_remover_nodes/param/test_params.yaml

# or provide parameters
ros2 run polygon_remover_nodes polygon_remover_node_exe \
--ros-args \
-r "points_xyzi":="/lidar_front/points_xyzi" \
-p working_mode:="Static" \
-p polygon_vertices:=[-8.0,-8.0,\
-8.0,8.0,\
8.0,0.0] \
-p will_visualize:=True
```

```yaml
/polygon_remover_nodes:
  ros__parameters:
    working_mode: "Static" # "Static" or "PolygonSub"
    # if working_mode is "Static"
    # x1, y1, x2, y2, ...
    polygon_vertices: [ -6.0, -6.0,
                        -6.0, 6.0,
                        6.0, 6.0,
                        6.0, -6.0 ] # Square
    will_visualize: True
```

## Inner-workings / Algorithms

<!-- If applicable -->
Explained in detail here:
[`polygon_remover`](/src/perception/filters/polygon_remover/design/polygon_remover-design.md)

## Error detection and handling

<!-- Required -->
Throws:

- `std::runtime_error("Please set working_mode to be one of: " + str_list_of_keys);`
  if `working_mode` is not set to either `Static` or `PolygonSub` and exits.

- `std::length_error("polygon_vertices has odd number of elements. "
  "It must have a list of x,y double pairs.");`
  if given vertex field count is not multiple of 2. (
  e.g. `[0.0,0.0, 3.0,0.0, 4.0,0.0, 7.0]`)

- Also, will throw other things if vertex count is less than 3
  (Explained in detail in
  [`polygon_remover`](/src/perception/filters/polygon_remover/design/polygon_remover-design.md))

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
https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/995
