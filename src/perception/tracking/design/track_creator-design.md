Track Creator {#tracking-track-creator}
===================

# Purpose / Use cases

Object tracking for an autonomous driving system involves associating new detections in every frame to tracks that the tracker keeps account of. New objects enter the field of view of the vehicle every frame and the perception sensor may also have false positives every frame. This means the tracker needs a module that will assess these detections that do not meet the matching criteria to be matched with any of the existing tracks. This module ideally needs to create new tracks only for detections that are from real objects which the tracker can take into its operation. 

# Design

[TrackCreator](@ref autoware::perception::tracking::TrackCreator) will initialize the appropriate the [CreationPolicyBase](@ref autoware::perception::tracking::CreationPolicyBase) interface and act as the interface for the tracker to create new tracks from detections that were not associated with any of the existing tracks.

Each track creation policy will need to derive from the [CreationPolicyBase](@ref autoware::perception::tracking::CreationPolicyBase) class and implement the `create()` function. Every policy implementation will also implement the `add_objects` function which is used to store the detections to be considered for track creation. 

This module is designed to be triggered to create new tracks after every new lidar clusters message. Hence it returns a `DetectedObjects` msg containing detections that are not used for creating new tracks (because of lack of association with objects from other modalities)

## Inner-workings / Algorithms

The logic used to create new tracks is controlled by [TrackCreationPolicy](@ref autoware::perception::tracking::TrackCreationPolicy).  

### LidarClusterOnly  
- Call `add_objects()` with the lidar clusters message and result from the lidar-track association. This method will go through every lidar cluster that was not associated to a track and store it internally  
- Call `create_tracks()`. This method will create one track per cluster that was stored internally in the previous step  
- Using this policy does not require a valid `GreedyRoiAssociator` object pointer. It can be initialized to nullptr
- This policy will not implement the `add_objects()` function for `ClassifiedROIArray` type message. Calling that will result in an exception being thrown

### LidarClusterIfVision
- Call `add_objects()` with the lidar clusters message and result from the lidar-track association. This method will go through every lidar cluster that was not associated to a track and store it internally  
- Call `add_objects()` with the vision detections message and result from the vision-track association. This method will go through every vision detection that was not associated to a track and store it internally. Every time this function is called a new vision detection message is created internally and pushed to a cache 
- Call `create_tracks()`. This method will first try to find a vision detection that is within `max_vision_lidar_timestamp_diff` from the lidar cluster msg stamp. If it finds such a message it will try to associate the lidar clusters and the vision detections. New tracks will be created only from lidar clusters that are matched with a vision detection
- Using this policy requires a valid `VisionPolicyConfig` struct object to be initialized in the `TrackCreatorConfig` struct object. 

## Parameters
- TrackCreationPolicy - Choose one of the available policies based on the above explanation
- Default variance & Noise variance - These are values to be used in the initialization of new [TrackedObject](@ref autoware::perception::tracking::TrackedObject) objects
- VisionPolicyConfig - This struct needs to be initialized for the LidarClusterIfVision policy
  - Transform from base_link to camera
  - iou_threshold - Minimum IOU value to math a lidar cluster and a vision detection
  - Instrinsic parameters of the camera
  - max_vision_lidar_timestamp_diff - Maximum difference between vision detections message stamp and lidar clusters message stamp to be considered for association

## Assumptions / Known Limitations
- This module assumes `create_tracks` will be called every time a lidar objects message is received
- All the functions in the module are assumed to be called from the same thread except for `add_objects(ClassifiedROIArray, ...)` which uses thread-safe data structures and can be called from a different thread 

# Future extensions / Unimplemented parts
- Support for radar detections
- Support for lidar detections

# Related issues:
- #1265 - Initial implementation
- #1120 - Add support for vision detection

