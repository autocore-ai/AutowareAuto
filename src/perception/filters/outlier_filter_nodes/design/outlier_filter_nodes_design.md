outlier_filter_nodes {#outlier-filter-nodes-package-design}
===========


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

The purpose of this package is to provide the ROS2 node interface for the outlier_filter library
functions. The functions of this library together with the node interface allows the user to apply
various outlier filter techniques on an input point cloud. 

The `outlier_filter_nodes` package contains the node executables for launching the:
 * `radius_search_2d_filter_node` - filter outliers using a radial search method in 2D
 * `voxel_grid_outlier_filter_node` - filter outliers using a voxel grid to remove points in
   outlying voxels

More information about the exact algorithm details can be found in @ref outlier-filter-algorithm.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

This package utilizes the `filter_node_base` and the `outlier_filter` libraries. Nodes extend from
the FilterNodeBase class and provide an interface between the ROS2 inputs and the library functions
from the `outlier_filter` package.


## Assumptions / Known limits
<!-- Required -->

The child classes are declared as `final` to ensure that no additional parameters are declared after
the calling the `set_param_callback()` method. 


## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

By inheriting from the `FilterNodeBase` the two nodes in this package by default subscribe to the
input `sensor_msgs::msg::PointCloud2` message and outputs `sensor_msgs::msg::PointCloud2` by
publishing the message. More information can be found in @ref filter-node-base-package-design.

Topic names are as follows:
 - `input`
 - `output`


### Parameters

From the FilterNodeBase the following parameter is required by both the outlier filter nodes
 - `max_queue_size` - (int) defines the maximum size of queues

Parameters specific to the nodes launched can be found described in @ref outlier_filter-package-design.
These will be required to be specified at launch.


## Error detection and handling
<!-- Required -->

N/A


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

N/A


# Related issues
<!-- Required -->

See:
 - Port from ArchitectureProposal (#917)
