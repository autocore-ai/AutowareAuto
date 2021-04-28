tracking_nodes {#tracking_nodes-package-design}
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
DetectedObject messages and Odometry messages with the same timestamp are received and given to the tracker implementation.


## Assumptions / Known limits
<!-- Required -->
Currently, due to using `message_filters` to implement timestamp matching, there is no warning when a DetectedObject message without corresponding Odometry is dropped.


## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
Input topics:
* "detected_objects"
* "odometry"

Output topics:
* "tracked_objects"


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