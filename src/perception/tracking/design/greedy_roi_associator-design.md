ROI Associator {#tracking-roi-associator}
============

Lidar and Radar detect 3D objects around the robot. If the visual measurements of these objects are available, then they can be associated to reinforce the 3D information.

# Purpose / Use cases

Tracking can be reinforced by fusing the data from multiple modalities that complement each 
other. Computer vision is one of such modalities that can offer certain strengths like being 
able to extract semantic information better than other sensors in properly lighted conditions.

In order to use the visual detections in conjunction with 3D detections, an associator that can associate the ROIs from images and the 3D detections is required.

# Design

[GreedyRoiAssociator](@ref autoware::perception::tracking::GreedyRoiAssociator) 
will associate objects in 3D with ROIs from an image by first projecting each 3D object to the image plane and then selecting the best match according to a match metric which is currently set to [IOUHeuristic](@ref autoware::perception::tracking::IOUHeuristic). The 3D detection can be from a lidar, radar or any other sensor that produces 3D output or it can be tracks that are already in the tracker.

## Inner-workings / Algorithms

See [Projection](@ref projection) document for more details on how the projection is executed.

The total complexity is expected to be determined by the association operation which has a 
worst case complexity of \f$O(N_TN_R(V_R+V_T))\f$ where \f$N_T\f$ is the number of 3D objects, \f$N_R\f$ is the number of ROIs, \f$V_R\f$ is the maximum number of vertices on a ROI and \f$V_T\f$ is the maximum number of vertices on a 3D object. The explanation behind the complexity is that, each object is compared to each ROI in a loop where the areas of the shapes are computed during the IoU computation. The area calculation has a complexity determined by the number of vertices in each shape, resulting  on a total complexity of \f$O(V_R+V_T)\f$ for each object-ROI comparison.

* Objects that are not on the image plane are not associated
* Objects that do not have matching ROI counterparts are not associated
* ROIs that do not have matching object counterparts are not associated
* To consider a ROI and an object to be a match, the computed IOU between them must be greater than a threshold.

## Inputs / Outputs / API

Inputs:
* Camera intrinsics
* Camera transformation
* A vector of `autoware::perception::tracking::TrackedObject`s
  or
  A vector of `autoware_auto_msgs::msg::DetectedObjects`
* `autoware_auto_msgs::msg::ClassifiedRoiArray`

Outputs:
* [AssociatorResult](@ref autoware::perception::tracking::AssociatorResult)


## Related issues

- #983: Integrate vision detections in object tracker 
- #1244: Associate lidar clusters with vision detections
