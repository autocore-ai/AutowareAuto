Running the EKF filter for localization {#ekf-localization-howto}
=======================================

@tableofcontents

# Ndt EKF smooth localization

This demo aims to show Kalman filter smoothing on top of NDT localization. The pose messages from
NDT get a covariance assigned, then get passed into the Kalman filter node. The smooth output is
then published as an Odometry topic and visualized in rviz2. This demo is tested with the Lexus car
in LG simulator on the parking lot map.

Before running the demo, ensure that the simulator is running:

Start simulation as described in @ref lgsvl.
Additionally, to configure LGSVL for this demonstration:

1. Maps: use this [map link](https://assets.dev.lgsvlsimulator.com/d5b8bb0b7f49875a8a4bbf83c50b3a4fe53779c7/environment_AutonomouStuff)
2. Vehicles: Select `ROS2 native` bridge type and paste the content of `AutowareAuto/src/tools/autoware_demos/config/svl/avp-sensors.json` into the `Sensors` text box
3. Simulations: In `General` tab, `Select Cluster = Local Machine` and untick any boxes.
In `Map & Vehicles` tab, ensure to untick `Run simulation in interactive mode`.
In `Traffic` tab, untick all selection.
The `Weather` tab is irrelevant.

Then, run the demo as follows:

```{bash}
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_demos ekf_ndt_smoothing_lgsvl.launch.py
```

The localization module will automatically locate the vehicle in the parking lot. You can use the controls to move the vehicle in the simulator and observe the red arrow move in tandem. For information about the control refer to @ref lgsvl-start-simulation. The following image shows the simulation working as expected:

\image html images/ekf-smoothing-localization.png width=50%
