Autonomous Valet Parking demonstration {#avpdemo}
=================================================

The Autonomous Valet Parking (AVP) demonstration uses Autoware.Auto to provide a valet parking service.

# Description of the demonstration

The AVP demonstration uses Autoware.Auto to provide the following functions:

1. Automatically drive a car from a pre-defined drop-off zone (e.g. the entrance to a carpark) to a
   parking spot indicated by an external system.
1. Park a car in a parking spot, starting in a lane near that parking spot.
1. Drive out of a parking spot.
1. Drive to a pre-defined pick-up zone (e.g. the exit from a carpark).
1. Automatically stop for obstacles while achieving the above.


# Prerequisites {#avpdemo-prerequisites}

To run this demo, you will need to provide the following.

- A point cloud map of a carpark, for localisation.
- A HDMap (vector map) of a carpark, for navigation.
- A simulated world of a carpark, if the demo will be run on a simulator.
- A simulated version of your car.

A sample carpark is available, based on a real single-level carpark in San Jose, California.

A sample car is available, based on a real-life vehicle used by many members of the Autoware Foundation.


# Hardware requirements

The AVP demo does not currently support running on hardware.


# Setup and launching (simulator)

The AVP demo can be run in the LGSVL simulator.
You can use the included sample carpark, or your own carpark.
If using your own carpark, you will need to provide a simulated world for it as described in [Prerequisites]{@ref avpdemo-prerequisites}.

1. [Install the ADE]{@ref installation-and-development-install-ade} and install the LGSVL simulator.
1. Configure the simulated world.
1. Configure the simulated car.
1. Follow the instructions for launching LGSVL to launch the simulator.
1. Do magic stuff to make it all work.


# Setup and launching (hardware)

The AVP demo does not currently support running on hardware.
