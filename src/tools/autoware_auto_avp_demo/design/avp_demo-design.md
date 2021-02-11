AVP demonstration package {#avp-demo-package-design}
===========

# Purpose / Use cases
The Automated Valet Parking (AVP) 2020 demo was held in October, 2020. This package is a storage location for launch and configuration files to allow the demo to easily be run with a single command.

# Design
Each milestone has a single launch file in the `launch/` folder which launches all required nodes to support the functions required to meet the milestone's goals.

The final milestone is milestone 3. For increased modularity, there are separate launch files to perform the demo in simulation (`ms3_sim.launch.py`) or in a real vehicle (`ms3_vehicle.launch.py`), with the common parts extracted to `ms3_core.launch.py`.

## Assumptions / Known limits
- The launch files should be installed/registered such that they can be called in the form `ros2 launch autoware_auto_avp_demo msX.launch.py`.
- Since the XML and YAML specifications for launch files are not yet available in Dashing, the launch files are written in Python.

## Inputs / Outputs / API
N/A.

## Inner-workings / Algorithms
This package doesn't implement any algorithms itself.

## Error detection and handling
Outside the scope of this package.

# Security considerations
N/A

# References / External links
- [Minutes](https://gitlab.com/autowarefoundation/autoware-foundation/-/wikis/ASWG-minutes-20200114) where this new package was discussed

# Future extensions / Unimplemented parts
None

# Related issues
- #246: Create location in stack for AVP 2020 Demo launch/configuration files
