Detected Object Associator {#tracking-detected-object-associator-design}
===========
This is the design document of the detected object associator that is used in the object tracking 
node


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->
The detected object associator module pairs up detections with tracks that are similar so that 
the new state of the tracks can be better estimated or new tracks can be created or existing  
tracks can be pruned.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
The detected object associator module uses Mahalanobis distance to compute how "similar" a track 
is to a detection. This cost is used to solve a hungarian assigner problem which returns a one to 
one association between detections and tracks. It is possible that some tracks/detection do not 
have any corresponding association.

## Assumptions / Known limits
<!-- Required -->
- Detections are assumed to be convex polygons with non zero area
- Detections are assumed to have their vertices ordered counterclockwise
- Each track in the tracks message has a vector of shapes to support articulated vehicles. The associator just uses the first shape in the vector for the purposes of association

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
### Inputs:
- List of detections
- List of tracks

### Parameters:
- Max area ratio between a track and its associated detection
- Max euclidean distance allowed between a track and its associated detection
- Boolean to control whether to use smallest side of detection as the threshold distance in cases where it is greater than the configured threshold distance

### Output
- Associations between track and detections  

### API:
- Call `assign()` with list of detection and tracks.  
  This will return a struct that contains,  
    1. Index of the associated detection for each track, or `AssociatorResult::UNASSIGNED` if a track could not be associated to any detection  
    1. Indices of all the detections that do not have an associated track  
    1. Indices of all the tracks that do not have an association detection

## Inner-workings / Algorithms
<!-- If applicable -->
- Figure out number of tracks and detections received and initialize the assigner appropriately  
- Loop through detections and tracks. For each pair check if they can be associated based on 
  the configured parameters  
- If a pair meets the gating parameters, compute the Mahalanobis distance between them and assign a weight to the pair in the assigner. Otherwise, ignore that pair    
- Call `assign()` function in the assigner  
- Loop through all the associations to figure out the tracks and detections with no 
  associations    

## Error detection and handling
<!-- Required -->
- If a track or detection shape does not meet the assumptions mentioned above, exception is thrown

# Future extensions / Unimplemented parts
<!-- Optional -->
- Consider many-to-one associations  
- Consider geometric similarity metrics  

# Related issues
<!-- Required -->
- #897 - Initial implementation
