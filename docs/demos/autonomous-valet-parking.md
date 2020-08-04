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


# Prerequisites {#avpdemo-prerequisites}

To run this demo, you will need to provide the following.

- A point cloud map of a carpark, for localisation.
- A HDMap (vector map) of a carpark, for navigation.
- A simulated world of a carpark, if the demo will be run on a simulator.
- A simulated version of your car.

A sample carpark is available, based on a real single-level carpark in San Jose, California.

A sample car is available, based on a real-life vehicle used by many members of the Autoware Foundation.


# Hardware requirements

The AVP demo can currently only be run in simulation.
To run the LGSVL simulator, you will need an Nvidia graphics card.


# Setup and launching (simulator)

The AVP demo can be run in the LGSVL simulator.
You can use the included sample carpark, or your own carpark.
If using your own carpark, you will need to provide a simulated world for it as described in @ref avpdemo-prerequisites.

1. @ref installation-and-development-install-ade and install the @ref lgsvl.
2. Follow the instructions on the @ref lgsvl page to configure a vehicle and simulation using the AutonomouStuff Parking Lot map.
3. Follow the instructions on the @ref lgsvl page to launch the simulator.
4. In a new terminal, run the launch file for Milestone 2.
The objectives and features of Milestone 2 can be found [here](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/24):
```console
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_auto_avp_demo ms2.launch.py
```


# Setup and launching (hardware)

The AVP demo does not currently support running on in-vehicle hardware.


# Related packages

- @subpage avp-demo-package-design
