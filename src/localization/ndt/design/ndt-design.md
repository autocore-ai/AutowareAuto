ndt
=============

# Purpose / Use cases

This package containes structures and functionality to allow localization with 3D lidar scans using Normal distribution transform matching. See (this article)[http://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf]
for further details of the implementation.

# Design

![ndt architecture](images/ndt_uml.svg)

## Algorithm Design

### Map

The space is divided into cells and each cells represents a single normal distribution. This is achieved by utilizing [voxel_grid](@ref autoware::perception::filters::voxel_grid) to
 associate the points with voxels while computing the mean and variance of each populated voxel. Voxels are then associated with a
 3D [spatial_hash](@ref autoware::common::geometry::spatial_hash) for allowing lookup.


## Assumptions / Known limits


## Inputs / Outputs / API

### Map
 Inputs:
 * Pointcloud

 Outputs:
 * Set of voxels given a point.

# Related issues

