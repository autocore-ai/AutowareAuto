avp_demo {#avp_demo-package-design}
===========

This is the design document for the `avp_demo` package.


# Purpose / Use cases
The Automated Valet Parking (AVP) 2020 demo will be held in late April, 2020. This package is a storage location for launch and configuration files to allow the demo to easily be run with a single command.

# Design
Each milestone will have a single launch file in the `launch/` folder which will launch all required nodes to support the functions required to meet the milestone's goals.

## Assumptions / Known limits
- The launch files should be installed/registered such that they can be called in the form `ros2 launch avp_demo msX.launch.py`.
- Since the XML and YAML specifications for launch files are not yet available in Dashing, the launch files will be written in Python.

## Inputs / Outputs / API
N/A.

## Inner-workings / Algorithms
N/A

## Error detection and handling
Outside the scope of this package.

# Security considerations
N/A

# References / External links
- [Minutes](https://gitlab.com/autowarefoundation/autoware-foundation/-/wikis/ASWG-minutes-20200114) where this new package was discussed

# Future extensions / Unimplemented parts
Most everything, at the moment.

# Related issues
- #246: Create location in stack for AVP 2020 Demo launch/configuration files
