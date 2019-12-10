ndt
=============

# Purpose / Use cases

This package contains structures and functionality to allow localization with 3D lidar scans using Normal distribution transform matching. See [this article](http://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf)
for further details of the implementation.

# Design

![ndt architecture](images/ndt_uml.svg)


## Map

### Algorithm Design

NDTMap, similar to [voxel_grid](@ref autoware::perception::filters::voxel_grid), is based on an `std::unordered_map` and
allows fast lookup given a point.

 NDT voxel map has 2 main types: [DynamicNDTMap](@ref autoware::localization::ndt::DynamicNDTMap) and
 [StaticNDTMap](@ref autoware::localization::ndt::StaticNDTMap).

[DynamicNDTMap](@ref autoware::localization::ndt::DynamicNDTMap) transforms a raw point cloud message into
an NDT map representation by passing the points to their corresponding voxels. Each voxel's centroid and covariance gets
computed with respect to the points that fall inside it. Covariance and centroid computation is done online with respect to [Welford's algorithm](https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance) .

Once [DynamicNDTMap](@ref autoware::localization::ndt::DynamicNDTMap) transforms a dense point cloud into voxels,
each voxel's centroid and covariance can be serialized into a `PointCloud2` message where the resulting point cloud is
sparse and only an intermediate representation of a transformed map. This point cloud can then be converted back into
an NDT map representation via [StaticNDTMap](@ref autoware::localization::ndt::StaticNDTMap)
 where no centroid/covariance is computed but only the points
are inserted into their corresponding voxels for lookup. Input point cloud is validated via
[validate_pcl_map()](@ref autoware::localization::ndt::validate_pcl_map) function before converting it into the map
representation.

### Assumptions / Known limits

### Inputs / Outputs / API
 Inputs:
 * Pointcloud

 Outputs:
 * Set of voxels given a point.


# Related issues

