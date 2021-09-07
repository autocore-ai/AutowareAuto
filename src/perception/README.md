Perception {#autoware-perception-design}
==========

# Domain Description

The `perception` sub-directory is divided into two sections: `filters` and `segmentation`.
The `filters` directory contains nodes and libraries which are part of the filtering pipeline for 
sensor data. 
Nodes in this directory are able to work as standalone filtering modules or cascaded with other 
filtering algorithms working on the same sensor data types. 
Packages under the `segmentation` directory enable sensor data to be grouped into clusters or 
objects of interest which can provide downstream procsses more information about the environment.

# Subpages

- @subpage autoware-perception-filters-design
- @subpage autoware-perception-segmentation-design
- @subpage tracking-architecture
- @subpage tracking-detected-object-associator-design
- @subpage running-tracker-with-vision
- @subpage tracking-nodes-design
- @subpage tracking-track-creator
- @subpage tracking-roi-associator
