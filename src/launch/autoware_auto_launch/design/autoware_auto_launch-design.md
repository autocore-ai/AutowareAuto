autoware_auto_launch {#autoware-auto-launch-package-design}
====================

This is the design document for the `autoware_auto_launch` package.


# Purpose / Use cases
<!-- Required -->
Parts of the AutowareAuto stack could be split into different pipelines allowing for modularity and component reuse between different ODDs and use-cases. 


# Design
<!-- Required -->
The launch files will be split into 9 different pipelines:
 * localization
 * mapping
 * perception
 * planning
 * sensors
 * simulator
 * system
 * vehicle
 * visualization

The launch files launches nodes with the same configuration as in the AVP demo, therefore,
launching all these launch files executes the AVP demo.


## Assumptions / Known limits
<!-- Required -->

The package splits the launching the AutowareAuto stack into smaller pipelines. For simplicity,
the launch files don't allow for internal input/output topic configuration. If this behaviour
is desired then a bespoke launch file should be created for this purpose or any external nodes
should have their topics remapped.


## Inputs / Outputs / API
<!-- Required -->

In each launch file, parameter files for nodes which require parameters are exposed as arguments.
The default for these parameter files are located in the `autoware_auto_launch` package under
`param`. In addition to parameter files, nodes which can be conditionally launched have been
exposed as arguments `with_rviz` and `with_obstacles`.


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
