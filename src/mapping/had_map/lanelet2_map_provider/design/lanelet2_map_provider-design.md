Lanelet2 map provider {#lanelet2-map-provider}
===========

This is the design document for the `lanelet2_map_provider` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->
This purpose of this node is to load and to distribute lanelet2 map data to other nodes as required.
Lanelet2 maps contain geometric, routing and semanctic information on road network, traffic regulations, sidewalks and parking spaces.
Nodes requiring specific map information should make a request to the map provider node for the geometric area and contents they need.
The map provider should then extract that information from a larger map and send the information to the relevant node.

Specific use cases (nodes requesting specific information from the
lanelet2 map provider node):
1. Global planner node - request road layout, lanelet level road network,
parking spaces and drivable areas for a bounded geographic region containing
the start and goal locations of the desired route, plus some additional region
allowing for non-direct routes.
The resulting information to send must be a
valid lanelet2 map to allow for construction of a routing graph.
2. Local planner - request road layout, lanelet level road network,
parking space and driveable areas of a local region around a specified location.
Location, orientation (?) and size of region should be given, as extent of
map needed depends on current velocity, type of maneuver etc.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
The design of the lanelet2 map provider is a typical ROS2 based 3 tier design:
- A lanelet2_map_provider base class provides a interface to the lanelet2 library
and is used to load a given map, and to extract geometric and primitive submaps
as binary messages
- A ROS2 node that uses the base class to load a specific map and defines a
service to respond to requests for  sub-maps

## Assumptions / Known limits
<!-- Required -->

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->


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
