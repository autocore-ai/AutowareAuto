# Changelog for Autoware.Auto

## 1.0.0 (Autonomous Valet Parking ODD)

This was the initial release of Autoware.Auto which specifically targeted the Autonomous Valet Parking Operational Design Domain.

### Major Features Added

* Vehicle Interface architecture
  * AutonomouStuff Speed and Steering Control software interface
  * LSGVL Simulator interface
* Vehicle control algorithms
  * Model Predictive Control algorithm with Ackermann-style vehicle support
  * Pure Pursuit control algorithm with Ackermann-style vehicle support
* Lidar-based perception pipeline
  * Transform lidar scans between frames and filtering/cropping
  * Fuse up to 7 lidar inputs
  * Ray-based Ground Filtering algorithm
  * Euclidean Clustering algorithm for object detection
  * Scan downsampling
  * Conversion to voxel-based scan representation
  * HD-Map-based scan filtering
* Lidar-based localization pipeline
  * NDT scan matching
  * Point cloud loader and publisher
  * Point cloud map generation
* HD-Map-based planning algorithms
  * Lanelet2 support
  * Map loader and server with query interface
  * Global planner to plan route using HD Map
  * Behavior planner to call individual sub-planners for different maneuvers
  * Lane sub-planner
  * Parking sub-planner
* Waypoint Record/Replay planning algorithm
  * No HD Map required
* Hardware drivers
  * Velodyne 3D lidar driver (from scratch)
  * Xsens GNSS/IMU driver (from scratch)
  * FLIR camera driver (based on Spinnaker SDK)
* Vehicle descriptions
  * Lexus RX 450h
* Utilities
  * Docker-based environment for both Autoware.Auto and LGSVL Simulator
  * Template-based ROS package creation
  * Common HAV algorithms and functions
  * Demos for many individual features and the Autonomous Valet Parking ODD
  * Joystick-based controller for vehicle interfaces
