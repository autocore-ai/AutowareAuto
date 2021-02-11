3D perception stack {#perception-stack-howto}
============

[TOC]

# Running the Autoware.Auto 3D perception stack

First, ensure that ADE is running and that everything is up to date. Open a terminal and type: `$ ade start --update`.

The Autoware.Auto 3D perception stack consists of a set of nodes necessary to compute and publish object bounding boxes. The minimal stack for doing so is:

2. [point_cloud_filter_transform_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/filters/point_cloud_filter_transform_nodes): Transforms output of the `velodyne_node` to a common frame.
3. [ray_ground_classifier_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/filters/ray_ground_classifier_nodes): Classifies point cloud points to indicate whether they belong to a ground or non-ground surface.
4. [euclidean_cluster_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/segmentation/euclidean_cluster_nodes): Clusters the non-ground points into object detections.

There are also optional nodes, not covered in this tutorial, that can be used to augment the stack:

1. [point_cloud_fusion](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/filters/point_cloud_fusion): Fuses point clouds from multiple sources into a single message. This is used currently to fuse the front and rear lidar data into a single message stream. This tutorial only uses the front lidar data.
2. [voxel_grid_nodes](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/filters/voxel_grid_nodes): This can be used to downsample point cloud data through a voxel grid representation. This tutorial does not perform downsampling.

To aid becoming familiar with the elements of the perception stack, the following subsections describe how to bring up the stack node by node without using the launch file. Follow the directions in sequence.

## Prerequisites

In order to run the perception stack, we need to open a visualizer, publish sensor data, and publish the "robot state," which is the transform tree describing the sensors positions with respect to the vehicle. The following subsections describe how to do this.

### Running the rviz2 visualizer

`rviz2` can be used to visualize perception data as it is published. To start the visualizer, open a new terminal, then:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ rviz2 -d /opt/AutowareAuto/share/autoware_auto_examples/rviz2/autoware_perception_stack.rviz
```
The rviz config has displays for all topics in this tutorial. As nodes are launched, they will be displayed in rviz. The checkboxes next to topic names can be checked and unchecked to toggle which perception outputs are visualized.

### Publishing sensor data

In order to bring up the perception stack, point cloud data needs to be published to the `/lidar_front/points_raw` topic. Several methods for doing this are given below.

1. Replaying recorded sensor data. To do this:
  1. Download the PCAP file [Dual VLP-16 Hi-Res pcap file](https://autoware-auto.s3.us-east-2.amazonaws.com/route_small_loop_rw.pcap).
  2. Move the downloaded file into your `adehome` folder.
  3. Replay the file using `udpreplay`:
```console
$ ade enter
ade$ udpreplay -r -1 route_small_loop_rw.pcap
```
  4. Launch the [velodyne_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/tree/master/src/drivers/velodyne_nodes) for the front lidar:

Dashing:

$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run velodyne_nodes velodyne_cloud_node_exe --model vlp16 __ns:=/lidar_front __params:=/opt/AutowareAuto/share/velodyne_nodes/param/vlp16_test.param.yaml
```

Foxy:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run velodyne_nodes velodyne_cloud_node_exe --model vlp16 --ros-args --remap __ns:=/lidar_front --params-file /opt/AutowareAuto/share/velodyne_nodes/param/vlp16_test.param.yaml
```

  5. Launch the [velodyne_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/tree/master/src/drivers/velodyne_nodes) for the rear lidar:

Dashing:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run velodyne_nodes velodyne_cloud_node_exe --model vlp16 __ns:=/lidar_rear __params:=/opt/AutowareAuto/share/velodyne_nodes/param/vlp16_test_rear.param.yaml
```

Foxy:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run velodyne_nodes velodyne_cloud_node_exe --model vlp16 --ros-args --remap __ns:=/lidar_rear --params-file /opt/AutowareAuto/share/velodyne_nodes/param/vlp16_test_rear.param.yaml
```
2. Running a simulator: To do this, see [Running the LGSVL Simulator along side Autoware.Auto](lgsvl.html)
3. Connecting to the sensor: To do this, update the IP address and port arguments in the param file for the [velodyne_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/tree/master/src/drivers/velodyne_nodes) and then launch the node:

Dashing:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run velodyne_nodes velodyne_cloud_node_exe --model vlp16 __ns:=/lidar_front __params:=/opt/AutowareAuto/share/velodyne_nodes/param/vlp16_test.param.yaml
```

Foxy:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run velodyne_nodes velodyne_cloud_node_exe --model vlp16 --ros-args --remap __ns:=/lidar_front --params-file /opt/AutowareAuto/share/velodyne_nodes/param/vlp16_test.param.yaml
```

\note
At this point, there exists a convenience launch file that can bring up the robot_state_publisher along with the rest of the perception stack using a single command.
You can either use the below launch file to bring up the stack, or continue on with the tutorial:

- When using PCAP data:
```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_demos lidar_bounding_boxes_pcap.launch.py
```
- When using the LGSVL simulator:
```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_demos lidar_bounding_boxes_lgsvl.launch.py
```

### Publishing the robot state

This node publishes the transform tree of the vehicle available. To do this:

- When using PCAP data:
```console
$ ade enter
ade$ ros2 run robot_state_publisher robot_state_publisher /opt/AutowareAuto/share/lexus_rx_450h_description/urdf/lexus_rx_450h_pcap.urdf
```
- When using the LGSVL simulator:
```console
$ ade enter
ade$ ros2 run robot_state_publisher robot_state_publisher /opt/AutowareAuto/share/lexus_rx_450h_description/urdf/lexus_rx_450h.urdf
```

## Bringing up the perception stack

Now that the prerequisites have been brought up, the perception stack can be launched.

### Run the point cloud filter transform node

This node transforms point clouds from the `velodyne_node` to a common frame. In a new terminal, do:

Dashing:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run point_cloud_filter_transform_nodes point_cloud_filter_transform_node_exe __ns:=/lidar_front __params:=/opt/AutowareAuto/share/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml __node:=filter_transform_vlp16_front  --remap points_in:=/lidar_front/points_raw
```

Foxy:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run point_cloud_filter_transform_nodes point_cloud_filter_transform_node_exe --ros-args --remap __ns:=/lidar_front --params-file /opt/AutowareAuto/share/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml --remap __node:=filter_transform_vlp16_front  --remap points_in:=/lidar_front/points_raw
```

![Autoware.Auto transformed points snapshot](autoware-auto-transformed-points.png)

### Run the ray ground classifier node

This node classifies point cloud points according to whether they are ground or non-ground. In a new terminal, do:

Dashing:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run ray_ground_classifier_nodes ray_ground_classifier_cloud_node_exe __params:=/opt/AutowareAuto/share/ray_ground_classifier_nodes/param/vlp16_lexus.param.yaml --remap points_in:=/lidar_front/points_filtered
```

Foxy:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run ray_ground_classifier_nodes ray_ground_classifier_cloud_node_exe --ros-args --params-file /opt/AutowareAuto/share/ray_ground_classifier_nodes/param/vlp16_lexus.param.yaml --remap points_in:=/lidar_front/points_filtered
```

![Autoware.Auto ray ground filter snapshot](autoware-auto-ray-ground-filter-smaller.png)

### Run the Euclidean cluster node

This node clusters non-ground points into objects and publishes bounding boxes. In a new terminal, do:

Dashing:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run euclidean_cluster_nodes euclidean_cluster_node_exe __params:=/opt/AutowareAuto/share/euclidean_cluster_nodes/param/vlp16_lexus_cluster.param.yaml --remap points_in:=/points_nonground
```

Foxy:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run euclidean_cluster_nodes euclidean_cluster_node_exe --ros-args --params-file /opt/AutowareAuto/share/euclidean_cluster_nodes/param/vlp16_lexus_cluster.param.yaml --remap points_in:=/points_nonground
```

![Autoware.Auto bounding boxes segmentation snapshot](autoware-auto-bounding-boxes-smaller.png)
