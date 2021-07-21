apollo_lidar_segmentation {#apollo-lidar-segmentation-design}
==================

# Design

## Convolutional Neural Networks (CNN) Segmentation

See the [original design](https://github.com/ApolloAuto/apollo/blob/3422a62ce932cb1c0c269922a0f1aa59a290b733/docs/specs/3d_obstacle_perception.md#convolutional-neural-networks-cnn-segmentation) by Apollo.
The paragraph of interest goes up to, but excluding, the "MinBox Builder" paragraph.
This package instead uses the Autoware BoundingBox message type for the bounding boxes.

Note: the parameters described in the original design have been modified and are out of date.

## Bounding Box

The lidar segmentation node establishes a bounding box for the detected obstacles.
Its corners form a rectangle aligned on the axes of the origin.

## API

For more details on the API, see the
[documentation](@ref autoware::perception::segmentation::apollo_lidar_segmentation::ApolloLidarSegmentation).

# Reference

Lidar segmentation is based off a core algorithm by [Apollo](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception.md), with modifications from [TierIV] (https://github.com/tier4/lidar_instance_segmentation_tvm) for the TVM backend.
