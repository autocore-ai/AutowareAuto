Mapping Design {#mapping-design}
=============

# Purpose / Use cases

Point cloud mapping aims to build a 3D point cloud map of an environment from sensor data that conveys 3D information about the 
surroundings of a perceiving agent, either directly like a lidar or indirectly as in the case of 2D visual SLAM.

## Input Use Cases

### Online data

This is a use case where a map is built as a vehicle operates, collects data and localizes itself in real time.
In this use case, the point cloud mapper aims to register measurements from online data sources 
that may include raw or processed sensor data and build a map of the environment in which to localize itself further.

Depending on the registration algorithm, the mapper may make use of additional sources of localization to provide initial estimates of robot's pose during
the mapping process.

In online map generation, the following components may be needed:
* An observation source for registration.
* Aiding localization source.

The output is in the following form:
* Transform / Pose for each registered input message.
* Map files.
* Associated meta data files.

Associated meta data files can contain the relevant `earth -> map` transform for the given map and a timestamp based on 
the timestamps of the received scans, pose estimates and the current system clock of the mapper, depending on the timestamp 
policy of the implementation.

### Offline data

#### Pre-recorded data:

Another common use case is to replay a pre-recorded stack of data relevant to the location to be mapped.
When pre-recorded data is used, pointcloud mapping can be done as in the case of online data. The only exception is that 
the mapper should be mindful that the actual time of map creation has diverged from the time origin of the data.

Running the mapping offline opens the possibility to run bulkier optimization algorithms to correct the map and register the input point clouds.

#### Simulated data:

In certain cases a map of a simulated environment may not be directly available and manual mapping may be necessary.
In this case the simulator should have the same data sources as in the case of online data. In this mapping mode, the environment
can get an exact mapping if the simulator provides ground truth data.


## Output Use Cases

### Localization

The registration portion of the mapper effectively works as an online source of localization and hence can fulfill the output use cases of a localizer
as specified in the [localization design document](@ref localization-design).

Alternatively, after an accurate map of an area is created and exported, either online or offline, this map can then be usef
for achieving relative localization in the given area later, without the need to map the environment from scratch.

### Visualization

Visualizing a pointcloud map can have various use cases including debugging or qualitatively assessing localization performance.

### Point cloud Transformation

Point clouds registered for the mapping process are transformed to the `map` frame after registration. This data can be published
to serve components that expect data in the map frame. This procedure can save on the re-transforming the raw point clouds
later by a transformer node.

### Modeling

The map of the environment can serve purposes other than localization or visualization. A 3D model of the environment
can be useful in numerous domains such as architecture, urban planning or building simulations and video games.

# Requirements

In the lights of the listed use cases, the mapping should satisfy the following functionality:

1. Register observations onto the map in progress.
2. Extend the map by appending the registered point clouds into the map.
3. Detect loop closures and correct the map or the trajectory representation.
4. Once the criterion for map exportation is triggered, write the map object into a file along with 
any metadata associated with it.

Requirement 2. implies appending data to a container. To keep the system predictable and static in memory,
the capacity of the map should be fixed and pre-allocated. Depending on the map representation, a struct like a voxel
grid can be used to reduce redundant or overlapping points.

Requirement 4. implies defining a policy of map creation to determine the content and the generation rate of the output maps.

Another thing to keep in mind is to prevent scalar overflows when the incoming point clouds diverge from the origin too much. 
In this case the `map` frame should be moved to keep the offsets from the origin reasonable.

The mapper should be modular and configurable as to allow switching between a real-time online configuration and an offline set up with a 
configuration that is more relaxed on the run-time constraints of an online mapping system.

# Design

## Inputs:
* Observations to register.
* Transforms in `/tf` topic.

## Outputs:
* Transforms/poses for registered observations.
* Transforms updating where the `map` frame is placed.
* Transformed point clouds.
* Map files in implementation defined format.
* Meta data files in implementation defined format.
* Map as a PointCloud2 message for visualization.

## Inner Workings: 
Given the use cases and requirements, the point cloud mapper is expected to be able to execute in accordance to
 the following workflow.

In the front-end: 

1. Allocate memory for containers.
2. For each received observation:
    1. Validate the message.
        1. Optionally select key-frames at this step.
    1. Register it to the existing map.
    1. Append the registered/reconstructed point cloud to the trajectory/map representation
    1. (Optional) Check for loop closure:
        1. Exit callback.
        1. Execute trajectory correction.
    1. Check for export criterion:
        1. Exit callback.
        1. Export the map.
3.  Publish the map and/or the byproducts of the process such as the registration output.
        
To make the map creation flexible and extendable, the mapper should allow flexibility and customization for the following
structs and functionality:

* Input type
* Map representation
* Trajectory representation
* Registration
* Loop-closure detection
* Trajectory correction
* Input preprocessing
* Output post-processing
* Map exportation
* Byproduct handling

