Object tracking architecture {#tracking-architecture}
==============================

# Purpose / Use cases
This document defines the high level architecture that the object tracker follows.

# Architecture  
![](images/tracking_architecture.png)

# Workflow  
The fundamental operation of a tracker is as follows (**bold** represents implementation-specific
items):

1. Update existing tracks to the most up-to-date coordinate frame
2. Receive observations
3. Associate observations to existing tracks
    1. Appropriately transform observations, if needed (this is unambiguous given a target frame)
    2. Update the tracks as needed (e.g. temporal update, **depends on motion model**)
    3. Compute the association weight/probability for each track-observation pair
  (**depends on association model**)
    1. Generate merge/split hypothesis (**depends on merge/split model**)
    2. Cache relevant information (this is implementation specific)
    3. Run the association algorithm (e.g. JVC, hungarian, GNN, PDA, **depends on association
    algorithm**)
    4. Get results
4. Update observed tracks
    1. Perform the appropriate transform of observation given an associated track (unambiguous given
    target frame, e.g. converting message type to internal representation)
    2. Apply the observation update to the track (**depends on state estimator and motion model**)
    3. Potentially perform track classification (**depends on track classification model**)
5. Resolve many-to-many matches
    1. Resolve merges (merges are potentially unambiguous given a decent state estimate model)
    2. Resolve splits (duplicate tracks, each with different single assignment)
6. Initialize new tracks
7. Update unassociated tracks
    1. Prune tracks if need be (**depends on pruning rule**)
8. Aggregate and transform the remaining tracks into the appropriate representation (unambiguous
given the message type, i.e. transform internal track type to the track message type)

# Description of submodules
This section will give a high level overview of each submodule mentioned in the workflow above. For a more detailed description refer to the design document specific to the submodule. The features described for each submodules may not be implemented yet. This serves as the guiding light.

## Association
The association module receives detections from lidar, radar and camera and uses the position information from those detections to find correspondences with tracks.  
Mahalanobis distance is used to compute a correspondence score between each track and detection.  
Gating based on area, classification and euclidean distance is done to reduce the number of possible track-detection pairs for which the score has to be calculated.   
Once the scores are calculated assignment is carried out by Hungarian algorithm. The algorithm is modified so that it can handle cases where every detection-track pair may not have a valid score.  

## Motion model  
Multiple motion models are used for each track and the estimates from them are combined by weighting them using classification information and covariance of the states.  

## Observation update  
Observations from each sensor modality are used according to the information that they are capable of providing. For example, most radar sensors can only provide a position of the target and not the shape. So, a radar measurement is used only to update the kinematics and position information and not the shape of the track.
