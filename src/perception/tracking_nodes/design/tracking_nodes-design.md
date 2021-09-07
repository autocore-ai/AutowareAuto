tracking_nodes {#tracking-nodes-design}
===========

This is the design document for the `tracking_nodes` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->
This is a ROS-layer wrapper around the `tracking` package.


# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
`DetectedObjects` and (optionally) `ClassifiedRoiArray` messages are time synchronized with `Odometry` 
or `PoseWithCovarianceStamped` messages which are then forwarded to the tracker implementation which 
updates the state of the tracks. The updated tracks are then acquired and published. 


## Assumptions / Known limits
<!-- Required -->
- If `use_vision` is set to True then the track creation policy is automatically updated as `LidarIfVision`
- If `use_vision` is set to True and no vision msg is received, no track will be published since 
  the creation policy requires association with vision

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
Input topics:
* "detected_objects"
* "classified_rois" (optional)
* "odometry"

Output topics:
* "tracked_objects"

Parameters:
* use_vision - Set this to true to subscribe to `ClassifiedRoiArray` topic. This also means
               `vision_association` section needs to be defined in the params file
* use_ndt - Set this to true to make tracker use `Odometry` msg from NDT. False will make
            tracker use `PoseWithCovarianceStamped` msg from `lgsvl_interface`

For a demo see @ref running-tracker-with-vision


## Inner-workings / Algorithms
<!-- If applicable -->


## Error detection and handling
<!-- Required -->


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->


# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->


# Related issues
<!-- Required -->
Issues #894, #1040
