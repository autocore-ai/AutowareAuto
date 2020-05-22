Localization nodes {#localization-nodes}
=============

# Purpose / Use cases

A node with boilerplate to operate a localizer and manage the input and output data flows is needed.

# Design

[RelativeLocalizerNode](@ref autoware::localization::localization_nodes::RelativeLocalizerNode) is a generic relative
localization node template that operates a [RelativeLocalizerBase](@ref autoware::localization::localization_common::RelativeLocalizerBase)
implementation, using a [PoseInitializerBase](@ref autoware::localization::localization_common::PoseInitializerBase) implementation.

* At each received observation message, the received message is registered in the localizer with the help of the fetched initial estimate and published.
* At each received map message, the map in the localizer is updated.



## Assumptions / Known limits

Since there are multiple callbacks, the node should be run in a single thread at any stage.

## Inputs / Outputs / API

Input:

- Map message
- Observation message
- Transform messages for initial estimate

Output:

- Output pose message


## Error detection and handling



## Security considerations


# Future extensions / Unimplemented parts


# Related issues

- #143 - Implement RelativeLocalizerBaseNode

