Lanelet2 Map for Autoware.Auto {#lanelet2-map-for-autoware-auto}
===
# Overview
Autoware.Auto uses the Lanelet2 format to define geometric and semantic information of lanes in the environment. For detailed information about the format, please refer to the official [Lanelet2 documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2).

In general, Autoware expects Lanelet2 maps to be created according to the upstream definition, with some minor changes. The following sections explain how objects and tags should be defined for Autoware.Auto.

@note The current explanation target the @ref avpdemo, and it could change with new ODDs.

# Primitives
For the AVP task, Autoware supports the following <a
href="https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LaneletPrimitives.md">Lanelet2
primitives</a>:
* Lanelet to to define driving lanes
* Area to define parking spots and parking access area
Note that <a href="https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/RegulatoryElementTagging.md">regulatory elements</a> (e.g. traffic lights, traffic signs, right of way, etc) are not used by Autoware.Auto as they are outside the AVP ODD.

## Defining Lanes
Lanes should be defined by Lanelet primitives according to the Lanelet2 format. The following information is utilized by Autoware.

### Information used in Autoware:
* participants: Only lanes for `vehicles` are supported by Autoware.Auto. This can be defined indirectly by `type` and `subtype` of the lanelet according to the [table in Lanelet2 documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LaneletAndAreaTagging.md#subtype-and-location) or directly by the `participants` tag.
* speed_limit: Autoware will not drive with speed that exceeds the value defined by this tag.

### Information that are **not** used by Autoware:
* lane change: In Lanelet2 format, the possibility of lane changes is determined by the type of the lanelet boundary. Autoware, however, does not support lane-change maneuvers and therefore it is ignored.
* bi-directional lanes: In the original Lanelet2 format, lanes can be defined as bi-directional lanes with the tag `one_way=no`. Instead, Autoware expects bidirectional lanes to be defined by two overlapping lanes with opposite directions. See the [AutonomouStuff parking-lot map](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/tools/autoware_demos/data/autonomoustuff_parking_lot.osm) as an example.

## Defining Parking Spots
Autoware expects parking spots to be defined as Area objects with the following tags:
* `subtype=parking_spot`: Areas with this tag will be treated as parking spots.
* `parking_accesses=<list of parking access ids>`: This defines the connected parking access areas that lead to the nearest lanes. The list of IDs should be separated by `,`(comma)

## Defining Parking Access Areas
Autoware expects the drivable region between lanes and parking spots to be defined as Area objects with the following tags:
* `subtype=parking_access`: Areas with this tag will be treated as parking access areas
* `parking_spots=<list of parking spot ids>`: This defines the connected parking spots to the parking access area. The list of IDs should be separated by `,`(comma).
* `ref_lanelet=<list of lanelet ids>`: This defines the connected lanes to the parking access area. The list of IDs should be separated by `,`(comma).
