LGSVL simulator {#lgsvl}
========

[TOC]

# LGSVL simulator: running the LGSVL simulator alongside Autoware.Auto

The following guide assumes that the LGSVL simulator will be run from inside an ADE container.

## Requirements

- ADE 4.1.0 or later. Follow the
[ADE installation instructions](https://ade-cli.readthedocs.io/en/latest/install.html) to install it
- NVidia graphics card
- If using Docker engine version 19.03 or later, [install Native GPU Support](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(Native-GPU-Support)).
- If using Docker engine with a version less than 19.03, either upgrade Docker or [install nvidia-docker2](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0)).

## Instructions

Install ADE as described in the
[installation section](installation-and-development.html#installation-and-development-install-ade):

Start ADE with the LGSVL volume:

```
$ cd ~/ade-home/AutowareAuto
$ source .aderc-lgsvl
$ ade start --update --enter
```

In the same terminal window, start the LGSVL simulator:

* `/opt/lgsvl/simulator`

Now start your favorite browser and go to [http://localhost:8080](http://localhost:8080) where
simulations can be configured.

**Note:** When running LGSVL Simulator in a Docker container, the "Open Browser..." button in the simulator window does not work.

### Vehicle Configuration

To configure the Lexus model, do the following in the browser:

- In the Vehicles tab look for `Lexus2016RXHybrid`. If not available, follow [these instructions](https://www.lgsvlsimulator.com/docs/vehicles-tab/#how-to-add-a-vehicle)
to add it and use the URL https://lgsvl-shared.s3-us-west-1.amazonaws.com/AWFLexus2016RXHybrid/vehicle_Lexus2016RXHybridApexAI

  - Click on the wrench icon for the Lexus vehicle
  - Change the bridge type to `ROS2`
  - In the `Sensors:` box, copy and paste the content of the `lgsvl-sensors.json` file in the root of the AutowareAuto repository

- Switch to the Simulations tab and click the `Add new` button

  - Enter a name and switch to the `Map & Vehicles` tab
  - Select a map from the drop down menu. If none are available follow [this guide](https://www.lgsvlsimulator.com/docs/maps-tab/#where-to-find-maps) to get a map.
  - Select the `Lexus2016RXHybrid` from the drop down menu. In the bridge connection box to the right enter the bridge address (default: `localhost:9090`)
  - Click submit

- Select the simulation and press the play button in the bottom right corner of the screen

The above steps are a modified version of the
[LGSVL documentation](https://www.lgsvlsimulator.com/docs/autoware-auto-instructions/#run-simulator-alongside-autowareauto)

#### Vehicle Appearance

By default, the vehicle "Lexus2016RXHybrid" uses a stock model of a VW wagon. To display the correct Lexus body model,
click the pencil icon on that vehicle and set the Vehicle URL to https://lgsvl-shared.s3-us-west-1.amazonaws.com/AWFLexus2016RXHybrid/vehicle_Lexus2016RXHybridApexAI.

![Lexus2016RXHybridEdit](lexus-2016-rx-hybrid-edit.png)

### ros2 web bridge

A version of `ros2 web bridge` is installed in the Autoware.Auto ade image.

If only perception is required, then this version may be used. In a new terminal window, run:

```
$ ade enter
ade$ rosbridge
```

#### From source

If vehicle control, or the bridging of `autoware_auto_msgs` is desired, then the ros2 web bridge
must be built from source. In a new terminal window, run:

```
$ ade enter
ade$ mkdir -p simulator_ws/src
ade$ cd ~/simulator_wr/src
ade$ git clone https://github.com/RobotWebTools/ros2-web-bridge -b 0.2.7
ade$ cd ros2-web-bridge
ade$ source /opt/AutowareAuto/setup.bash
ade$ npm install
```

Once the install is finished run the following in the same terminal window:

```
ade$ node bin/rosbridge.js
```

The act of running the ros2 web bridge with the appropriate packages sourced will allow the web
bridge to bridge these non-standard messages.

### Bridging with Autoware.Auto

LGSVL uses conventions which are not directly aligned with ROS 2 conventions.

For example:
- Left handed coordinate system with heading zero at +y
- Positive steering/wheel angle results in clockwise rotation of the vehicle

To make these conventions consistent, the `lgsvl_interface` is provided.

To run the `lgsvl_interface`, enter the following in a new terminal window:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run lgsvl_interface lgsvl_interface_exe __params:=/opt/AutowareAuto/share/lgsvl_interface/lgsvl.param.yaml
```

Launch scripts are also provided for convenience. For example for a joystick control demo, run the following in a new terminal window:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch lgsvl_interface lgsvl_joystick.launch.py
```

\warning The following demo currently does not work due to a bug in the launch scripts

For an example of using `VehicleControlCommand` with LGSVL, run the following demo in a new terminal window:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch lgsvl_interface lgsvl_vehicle_control_command.launch.py
```

The full list of behaviors the `lgsvl_interface` implements is:
1. Converts GPS-Odometry sensor to `VehicleKinematicState` and `TFMessage` (on `/tf`) with:
    1. The orientation corrected (right-handed system), and the vehicle frame with x forward
    2. Position set to zero with the first ground truth position. This makes the `/odom` frame local
    3. This behavior can be disabled by setting the simulator odometry topic to "null"
2. Converts control inputs with CCW positive rotations to the CCW negative inputs the LGSVL
simulator expects
3. Provides a mapping from `VehicleControlCommand` to the `RawControlCommand` LGSVL expects via
parametrizable 1D lookup tables


## Troubleshooting

### No data are being sent through to ROS and/or the bridge status in the simulator shows "Disconnected" 

To check that data are arriving on each of the required topics, run the following in a new terminal window:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 topic hz /raw_command
ade$ ros2 topic hz /vehicle_cmd
```

If data are not arriving on one of these topics, then the stack did not start up correctly, or there
is a configuration problem. Stop the vehicle interface and joystick controller and inspect the logs
to determine which is the case.

**Note:** Some Linux installations come with the default route for `localhost` set to an IPv6 interface
instead of IPv4. The LGSVL simulator does not currently support IPv6 so a modification must be made.
In the Simulation configuration, change the ROS2 Bridge address from `localhost:9090` to `127.0.0.1:9090`
and restart the simulation. This will ensure that the simulator is using IPv4.

### The brake/throttle/steering does not work

The joystick control mapping is not deterministic. It is occasionally necessary to modify the axis
mapping.

First, with the joystick controller running, verify that you can see the raw messages by running
the following:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 topic echo /joy
```

Next, actuate the appropriate axis on the vehicle controllers to determine which buttons and joy
sticks correspond to which indices in the `Joy` message.

Update the `joystick_vehicle_interface/param/logitech_f310.defaults.param.yaml` appropriately, or
make a copy.

### There are no data on the /joy topic

Ensure that `/dev/input/js0` is available from within ade.

If it is not available, restart `ade`, ensuring that it is run with the `--privileged` flag, or by
appropriately mounting the given device.

### The vehicle still does not move

First, ensure the whole stack is running properly, and is appropriately configured. See the section
above titled "No data are being sent through to ROS."

Next, ensure there are data on the `/joy` topic. If this is not the case, refer to the appropriate
question.

If data are available on all topics, and the vehicle is still not moving, ensure that the port
(9090 by default) needed by the web bridge is exposed by running the following command:

```
$ ss -lnt
```

If the port is not exposed, make sure ade exposes the port on local loopback by restarting ADE with
the added arguments:

```
$ ade start <ade arguments> -- <other docker arguments> -p 127.0.0.1:<PORT>:<PORT>
```

Finally, if the vehicle still does not move, ensure you are running a version of ros2 web bridge
built against `autoware_auto_msgs`.
