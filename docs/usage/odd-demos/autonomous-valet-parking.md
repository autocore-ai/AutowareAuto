Autonomous Valet Parking demonstration {#avpdemo}
=================================================

The Autonomous Valet Parking (AVP) demonstration uses Autoware.Auto to provide a valet parking service in the parking lot ODD.

![Autonomous valet parking](images/valet_parking.jpeg)

# Description of the demonstration

The AVP demonstration uses Autoware.Auto to provide the following functions:

1. Automatically drive a car from a pre-defined drop-off zone (e.g. the entrance to a carpark) to a
   parking spot indicated by an external system.
2. Park a car in a parking spot, starting in a lane near that parking spot.
3. Drive out of a parking spot.
4. Drive to a pre-defined pick-up zone (e.g. the exit from a carpark).
5. Automatically stop for obstacles while achieving the above.

# System architecture for the AVP ODD

The system architecture that was developed to address the AVP ODD in Autoware.Auto is given below:

![Autoware.Auto AVP Architecture](images/AVP_Architecture.png)

# Prerequisites {#avpdemo-prerequisites}

To run this demo, you will need to provide the following.

- A point cloud map of a carpark, for localisation.
- A HDMap (vector map) of a carpark, for navigation.
- A simulated world of a carpark, if the demo will be run on a simulator.
- A simulated version of your car.

A sample carpark is available, based on a real single-level carpark in San Jose, California.

A sample car is available, based on a real-life vehicle used by many members of the Autoware Foundation.


# Hardware requirements

## Physical demo

The physical AVP demo was tested with the following hardware:

- 2 Velodyne VLP-32Cs
- an AutonomouStuff Spectra industrial PC.
- an autonomouStuff PACMod drive-by-wire system
- the AutonomouStuff Speed and Steering Control (SSC) software.

## Simulation demo

The simulation AVP demo was tested with hardware that satisfies requirements for the LGSVL simulator.
To run the LGSVL simulator, you will need an NVIDIA graphics card. Additional information about requirements can be found [here](https://www.lgsvlsimulator.com/docs/faq/#what-are-the-recommended-system-specs-what-are-the-minimum-required-system-specs).

# Setup and launching (simulator)

The AVP demo can be run in the LGSVL simulator.
You can use the included sample carpark, or your own carpark.
If using your own carpark, you will need to provide a simulated world for it as described in @ref avpdemo-prerequisites.

1. @ref installation-and-development-install-ade and install the @ref lgsvl.
2. Follow the instructions on the @ref lgsvl page to configure a vehicle and simulation using the AutonomouStuff Parking Lot map (AS Parking Lot).
3. Follow the instructions on the @ref lgsvl page to launch the simulator.
4. In a new terminal, run the launch file for Milestone 3:

```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_auto_avp_demo ms3_sim.launch.py
```

# Setup and launching (hardware)

Using the hardware that is defined in the Physical Demo section above, the demonstration can be run on physical hardware only in parking lots for which a Lanelet2 map and lidar map are available.

To run the demonstration on a physical vehicle using your own maps:

1. Replace the paths to the `.pcd` and `.yaml` files for the PCD map in the `src/tools/autoware_auto_avp_demo/param/map_publisher_vehicle.param.yaml` with paths to your own map files.
2. Replace the path to the `.osm` file for the Lanelet2 map in the `src/tools/autoware_auto_avp_demo/param/lanelet2_map_provider.param.yaml` with the path to your own map file.

Whether using your own maps or the existing ones:

1. @ref installation-and-development-install-ade
2. In a new terminal, run the launch file for Milestoe 3:

```console
$ ade enter
ade$ source /opt/AuotwareAuto/setup.bash
ade$ ros2 launch autoware_auto_avp_demo ms3_vehicle.launch.py
```


# Related packages

- @subpage avp-demo-package-design
