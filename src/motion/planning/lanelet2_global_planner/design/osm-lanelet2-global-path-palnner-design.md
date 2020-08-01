OSM Planner {#osm-planner}
=============

# Purpose / Use cases

This package provides a global path planner with the lanelet2 openstreet map (osm) xml. It plans a shortest path/lane  
from a current location to the final parking spot following traffic rules define in the map. The path is  
given as an array of the lane id from the starting to the goal position.  

It is note that the global path is the center lane line and include the path from/to the parking spot through the drivable  
area defined in the map.  


# Design

The path(lane) planning utilised the lanelet2 api to get the shortest path from given the starting and  
the goal location (x,y,z). The planning steps include,

1. Find the drop-off and parking location id from the given position e.g. current location from a localiser node (find_nearparking_from_point)   
2. Find the start and goal lane id from the parking and lane association defined in the osm map (find_lane_id)  
3. Find the drivable area path id that connects the drop-off and parking location to the lane route (to be implemented)  
4. Find the shortest path graph search using lanelet2 api (get_lane_route)  
5. Concatenate the shortest path from 4 with the drivable parking path to the lane route from 3. (to be implemented)   


## Assumptions / Known limits

- The osm map must have the parking id to the lane association
- The osm map must have the drivable area attribute that associate the parking id to the lane id
- The path needs to be plan from/to the parking spot only at this stage


## Inputs / Outputs / API

Inputs:

- Binary map
- GPS location (lat, lon, alt) to convert the map geo-location to utm coordinate
- The start/goal position

Outputs:
- Lane id array

## Complexity

Right now, searching the parking spot from the given location is `O(n-1)` in space, std::min_element
(https://en.cppreference.com/w/cpp/algorithm/min_element#Complexity)


# Related issues

- #470 - Map loader
