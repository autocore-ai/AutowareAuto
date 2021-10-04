3D perception stack {#perception-stack-howto}
============

@tableofcontents

# Overview

This demo aims to explain the general process for running the following perception nodes. The Autoware.Auto 3D perception stack consists of a set of nodes necessary to compute and publish object bounding boxes. The minimal stack for doing so is:

1. [point_cloud_filter_transform_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/filters/point_cloud_filter_transform_nodes): Transforms output of the `velodyne_node` to a common frame. Documentation can be found here: @ref point-cloud-filter-transform-nodes.
2. [ray_ground_classifier_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/filters/ray_ground_classifier_nodes): Classifies lidar points to indicate whether they belong to a ground or non-ground surface. Documentation can be found here: @ref ray-ground-classifier-design.
3. [euclidean_cluster_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/segmentation/euclidean_cluster_nodes): Clusters the non-ground points into object detections. Documentation can be found here: @ref euclidean-cluster-design.

There are also optional nodes, not covered in this tutorial, that can be used to augment the stack:

1. [point_cloud_fusion](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/filters/point_cloud_fusion): Fuses point clouds from multiple sources into a single message. This is used currently to fuse the front and rear lidar data into a single message stream. This tutorial only uses the front lidar data. Documentation can be found here: @ref point-cloud-fusion-nodes.
2. [voxel_grid_nodes](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/perception/filters/voxel_grid_nodes): This can be used to down-sample point cloud data through a voxel grid representation. This tutorial does not perform down-sampling. Documentation can be found here: @ref voxel-grid-filter-design.

# Launch Perception Nodes

## Prerequisites

This demo assumes that instructions are run inside the ADE environment. For more information for installation see @ref installation-and-development-install-ade. Depending on which mode is choosen to generate data, a different `.aderc` file might need to be used ensure the correct environment is configured.

## Generating Sensor Data

To run the perception stack, sensor data will need to be generated and publish along with the corresponding "robot state"  that is used to determine  the sensor positions with respect to the vehicle. This section covers three different ways of achieving this:
 * Running the simulator
 * Replaying sensor data
 * Connecting to the physical sensor

### Running the Simulator

1. Enter ADE using the LGSVL configuration:
  ```{bash}
  $ ade --rc .aderc-lgsvl start --update --enter
  ```
2. See [Running the SVL Simulator along side Autoware.Auto](lgsvl.html)
3. Publish the robot state description:
  ```{bash}
  $ ade enter
  ade$ ros2 run robot_state_publisher robot_state_publisher /opt/AutowareAuto/share/lexus_rx_450h_description/urdf/lexus_rx_450h.urdf
  ```

### Replaying Sensor Data

1. Download the PCAP file [Dual VLP-16 Hi-Res pcap file](https://autoware-auto.s3.us-east-2.amazonaws.com/route_small_loop_rw.pcap).
2. Enter ADE using the default script with extra flags to properly run RViz
  ```{bash}
  $ ade --rc .aderc start --update --enter -- --net=host --privileged
  ```
3. Move the downloaded file into your `adehome` folder.
4. Replay the file using `udpreplay`:
  ```{bash}
  $ ade enter
  ade$ udpreplay -r -1 route_small_loop_rw.pcap
  ```
5. Launch the [velodyne_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/tree/master/src/drivers/velodyne_nodes) for the front lidar:
  ```{bash}
  $ ade enter
  ade$ source /opt/AutowareAuto/setup.bash
  ade$ ros2 run velodyne_nodes velodyne_cloud_node_exe --model vlp16 --ros-args --remap __ns:=/lidar_front --params-file /opt/AutowareAuto/share/velodyne_nodes/param/vlp16_test.param.yaml
  ```
6. Launch the [velodyne_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/tree/master/src/drivers/velodyne_nodes) for the rear lidar:
  ```{bash}
  $ ade enter
  ade$ source /opt/AutowareAuto/setup.bash
  ade$ ros2 run velodyne_nodes velodyne_cloud_node_exe --model vlp16 --ros-args --remap __ns:=/lidar_rear --params-file /opt/AutowareAuto/share/velodyne_nodes/param/vlp16_test_rear.param.yaml
  ``` 
7. Publish the robot state description: 
  ```{bash}
  $ ade enter
  ade$ ros2 run robot_state_publisher robot_state_publisher /opt/AutowareAuto/share/lexus_rx_450h_description/urdf/lexus_rx_450h_pcap.urdf
  ```

### Connecting to the Physical Sensor

1. Enter ADE using the default script with extra flags to properly run RViz
  ```{bash}
  $ ade --rc .aderc start --update --enter -- --net=host --privileged
  ```
2. To do this, update the IP address and port arguments in the parameter file for the [velodyne_node](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/tree/master/src/drivers/velodyne_nodes) and then launch the node:
  ```{bash}
  $ ade enter
  ade$ source /opt/AutowareAuto/setup.bash
  ade$ ros2 run velodyne_nodes velodyne_cloud_node_exe --model vlp16 --ros-args --remap __ns:=/lidar_front --params-file /opt/AutowareAuto/share/velodyne_nodes/param/vlp16_test.param.yaml
  ```
## Launch Visualization

`rviz2` can be used to visualize perception data as it is published. To start the visualizer, open a new terminal, then:

  ```{bash}
  $ ade enter
  ade$ source /opt/AutowareAuto/setup.bash
  ade$ rviz2 -d /opt/AutowareAuto/share/autoware_auto_examples/rviz2/autoware_perception_stack.rviz
  ```

The rviz config has displays for all topics in this tutorial. As nodes are launched, they will be displayed in rviz. The check-boxes next to topic names can be checked and unchecked to toggle which perception outputs are visualized.

## Launch Perception Nodes

1. Launch the `point_cloud_filter_transform_node` node. This node transforms point clouds from the `velodyne_node` to a common frame. In a new terminal, do:
  ```{bash}
  $ ade enter
  ade$ source /opt/AutowareAuto/setup.bash
  ade$ ros2 run point_cloud_filter_transform_nodes point_cloud_filter_transform_node_exe --ros-args --remap __ns:=/lidar_front --params-file /opt/AutowareAuto/share/point_cloud_filter_transform_nodes/param/vlp16_sim_lexus_filter_transform.param.yaml --remap __node:=filter_transform_vlp16_front  --remap points_in:=/lidar_front/points_xyzi
  ```
  ![Autoware.Auto transformed points snapshot](autoware-auto-transformed-points.png)

2. Launch the `ray_ground_classifier_node` node. This node classifies point cloud points according to whether they are ground or non-ground. In a new terminal, do:
  ```{bash}
  $ ade enter
  ade$ source /opt/AutowareAuto/setup.bash
  ade$ ros2 run ray_ground_classifier_nodes ray_ground_classifier_cloud_node_exe --ros-args --params-file /opt/AutowareAuto/share/ray_ground_classifier_nodes/param/vlp16_lexus_pcap.param.yaml --remap points_in:=/lidar_front/points_filtered
  ```
  ![Autoware.Auto ray ground filter snapshot](autoware-auto-ray-ground-filter-smaller.png)

3. Launch the `euclidean_cluster_node` node. This node clusters non-ground points into objects and publishes bounding boxes. In a new terminal, do:
  ```{bash}
  $ ade enter
  ade$ source /opt/AutowareAuto/setup.bash
  ade$ ros2 run euclidean_cluster_nodes euclidean_cluster_node_exe --ros-args --params-file /opt/AutowareAuto/share/euclidean_cluster_nodes/param/vlp16_lexus_cluster.param.yaml --remap points_in:=/points_nonground
  ```
  ![Autoware.Auto bounding boxes segmentation snapshot](autoware-auto-bounding-boxes-smaller.png)

## Convenience Launch Files

To simplify the process of launching these nodes there exists a convenience launch file that can bring up the `robot_state_publisher` along with the rest of the perception stack using a single command. To bring up the perception stack, use the following launch files:

### PCAP Sensor Data

\note See above steps for replaying the sensor data if you don't have a copy of the PCAP file.

1. Replay the file using `udpreplay`:
  ```{bash}
  $ ade enter
  ade$ udpreplay -r -1 route_small_loop_rw.pcap
  ```
2. Enter ADE and run the launch file
  ```{bash}
  ade enter
  ade$ source /opt/AutowareAuto/setup.bash
  ade$ ros2 launch autoware_demos lidar_bounding_boxes_pcap.launch.py
  ```

### SVL Simulator

\note If the SVL Simulator is already running, skip step 1.

1. See [Running the SVL Simulator along side Autoware.Auto](lgsvl.html)
2. Enter ADE and run the launch file
  ```{bash}
  $ ade enter
  ade$ source /opt/AutowareAuto/setup.bash
  ade$ ros2 launch autoware_demos lidar_bounding_boxes_lgsvl.launch.py
  ```
