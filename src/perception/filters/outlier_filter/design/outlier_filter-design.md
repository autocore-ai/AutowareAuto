outlier_filter {#outlier_filter-package-design}
===========


# Purpose / Use cases

The `outlier_filter` package is a library package containing the implementations of the three
outlier filters from the TierIV [AutowareArchitectureProposal repository](https://github.com/tier4/AutowareArchitectureProposal.iv):
 * `radius_search_2d_filter` - uses PCL functions to radially search a point for surrounding neighbors 
 * `voxel_grid_outlier_filter` - uses voxels to determine if a particular voxel contains outliers
 * `ring_filter` (Not Implemented Yet) - uses ring information from the lidar to determine if a point is an outlier  


# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

The algorithm originally coupled with the ROS2 Node interfaces has been separated to create two new
packages following the package design guidelines in the AutowareAuto. This library package contains
the main implementation of the filtering algorithms for the three outlier filter types.


## Assumptions / Known limits
<!-- Required -->

### Discretization

The `voxel_grid_outlier_filter` implementation using discretized voxel to determine whether a point is an
outlier. Due to the nature of discretized algorithms, a small approximation error could be
introduced. For applications where the reduction of approximation error is important, alternative
outlier filter implementations such as the `radius_search_2d_filter` should be used instead.

### Radius Search 2D Only

The `radius_search_2d_filter` implementation is only in the 2d space. More details in the sections below.

### Controlling Access To Parameters

As parameters may be changed during runtime, the node class using the filter classes will need to be
control access to the parameters via mutexes. The functions that require guarding are
`update_parameters` and `filter`.


## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
### Constructor

Parameters used by the filtering algorithm should be initially passed into the filter class through
the class constructor. A typical constructor for filter class in this package would be in the
form:

```{cpp}
explicit OUTLIER_FILTER_PUBLIC src/perception/filters/outlier_filter/src/voxel_grid_outlier_filter.cpp(int input_a, int input_b);
```

Where `input_a` and `input_b` are the exhaustive list of parameters required for the filter
functionality. The name of the filter should be appended with the suffix `Filter`.


#### radius_search_2d_filter

The following parameters are required by the `radius_search_2d_filter`:
 * `search_radius_` - radius around point where neighbors are searched (Type: `double`, Unit: meters)
 * `min_neighbors_` - minimum number of neighbors required for point to not be considered an outlier (Type: `int`)


#### voxel_grid_outlier_filter

The following parameters are required by the `voxel_grid_outlier_filter` class constructor:
 * `voxel_size_x` - voxel leaf size in the x-axis (Type: `float`, Unit: meters)
 * `voxel_size_y` - voxel leaf size in the y-axis (Type: `float`, Unit: meters) 
 * `voxel_size_z` - voxel leaf size in the z-axis (Type: `float`, Unit: meters)
 * `voxel_points_threshold` - minimum number of points in voxel for a voxel to be valid (Type: `int`)


### Modifying Parameters

The classes allow for parameters to be modified during run-time. The Node class using the filter
library should call the `update_parameters` method and pass in all the parameters; for example:

```{cpp}
std::mutex mutex_;
float search_radius = 1.0;
int min_neighbors = 5;
auto filter = std::make_shared<RadiusSearch2DFilter>(search_radius, min_neighbors);

// Perform some update on the parameters
search_radius = 1.5;
{
    std::lock_guard<std::mutex> lock(mutex_);
    filter->update_parameters(search_radius, min_neighbors);
}
```


### Filtering Function

Each of the filter classes should contain some filter activation function which will run the filter
on an input point cloud.

```{cpp}
void OUTLIER_FILTER_PUBLIC filter(
    const pcl::PointCloud<pcl::PointXYZ> & input,
    pcl::PointCloud<pcl::PointXYZ> & output);
```

Depending on the filtering method the point type used in the input may differ. The filtered point
cloud is stored in `output`.


## Inner-workings / Algorithms {#outlier-filter-algorithm}
<!-- If applicable -->
### radius_search_2d_filter

The `radius_search_2d_filter` filtering method is straightforward. The input point cloud is
flattened by converting the PointXYZ to PointXY. It utilizes the [PCL `Search` object]
(https://pointclouds.org/documentation/classpcl_1_1search_1_1_search.html) to search a radius
(defined by `search_radius_`) around the point of interest for neighboring points. If the minimum
number of neighboring points is found, the original point is added to the output point cloud.


### voxel_grid_outlier_filter

The `voxel_grid_outlier_filter` filtering method using the PCL library `VoxelGrid` object to discretize the
point cloud into equi-sized voxels containing a certain number of points. Then the 3D grid is
searched to determine if a point lies within a voxel. Points returning an existing voxel are not
considered outliers and are added to the output point cloud.


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

# External References

## radius_search_2d_filter

The original implementation of the `radius_search_2d_filter` can be found in the TierIV
AutowareArchitectureProposal repository:
* [`radius_search_2d_outlier_filter_nodelet.hpp`](https://github.com/tier4/AutowareArchitectureProposal.iv/blob/ros2/sensing/preprocessor/pointcloud/pointcloud_preprocessor/include/pointcloud_preprocessor/outlier_filter/radius_search_2d_outlier_filter_nodelet.hpp)
* [`radius_search_2d_outlier_filter_nodelet.cpp`](https://github.com/tier4/AutowareArchitectureProposal.iv/blob/ros2/sensing/preprocessor/pointcloud/pointcloud_preprocessor/src/outlier_filter/radius_search_2d_outlier_filter_nodelet.cpp)


## voxel_grid_outlier_filter

The original implementation of the `voxel_grid_outlier_filter` can be found in the TierIV
AutowareArchitectureProposal repository:
* [`voxel_grid_outlier_filter_nodelet.hpp`](https://github.com/tier4/AutowareArchitectureProposal.iv/blob/ros2/sensing/preprocessor/pointcloud/pointcloud_preprocessor/include/pointcloud_preprocessor/outlier_filter/voxel_grid_outlier_filter_nodelet.hpp)
* [`voxel_grid_outlier_filter_nodelet.cpp`](https://github.com/tier4/AutowareArchitectureProposal.iv/blob/ros2/sensing/preprocessor/pointcloud/pointcloud_preprocessor/src/outlier_filter/voxel_grid_outlier_filter_nodelet.cpp)


# Related issues
<!-- Required -->

See:
 - Port from ArchitectureProposal (#917)
