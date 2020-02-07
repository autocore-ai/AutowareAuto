LGSVL simulator {#lgsvl}
========

[TOC]

# LGSVL simulator: running the LGSVL simulator alongside Autoware.Auto

The following guide assumes that the LGSVL simulator will be run from inside an ADE container.

## Requirements

- ADE 4.1.0 or later. Follow the
[ADE installation instructions](https://ade-cli.readthedocs.io/en/latest/install.html) to install it

## Instructions

Install ADE as described in the
[installation section](installation-and-development.html#installation-and-development-install-ade):

Start ADE with the LGSVL volume:

```
$ cd ~/ade-home/AutowareAuto
$ source .aderc-lgsvl
$ ade start --update --enter
```

Start the LGSVL simulator:

* `/opt/lgsvl/simulator`

Now start your favorite browser and go to [http://localhost:8080](http://localhost:8080) where
simulations can be configured.

Follow the instructions in the
[LGSVL documentation](https://www.lgsvlsimulator.com/docs/autoware-auto-instructions/#run-simulator-alongside-autowareauto)
to configure the Lexus model to use the ROS 2 bridge.

For a sensor json configuration that works out of the box, copy/paste the file in the root of the
AutowareAuto repository, `lgsvl-sensors.json`, into the vehicle configuration dialogue box.

### ros2 web bridge

A version of `ros2 web bridge` is installed in the Autoware.Auto ade image.

If only perception is required, then this version may be used:

```
$ ade enter
ade$ rosbridge
```

#### From source

If vehicle control, or the bridging of `autoware_auto_msgs` is desired, then the ros2 web bridge
must be built from source:

```
$ ade enter
ade$ mkdir -p simulator_ws/src
ade$ cd ~/simulator_wr/src
ade$ git clone https://github.com/RobotWebTools/ros2-web-bridge -b 0.2.7
ade$ cd ros2-web-bridge
ade$ source /opt/AutowareAuto/setup.bash
ade$ npm install
```

And then run:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ cd simulator_ws/src/ros2-web-bridge
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

To run the `lgsvl_interface`:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run lgsvl_interface lgsvl_interface_exe __params:=/opt/AutowareAuto/lgsvl_interface/share/lgsvl.param.yaml
```

Launch scripts are also provided for convenience. For example for a joystick control demo, run:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch lgsvl_interface lgsvl_joystick.launch.py
```

For an example of using `VehicleControlCommand` with LGSVL, run the following demo:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch lgsvl_interface lgsvl_vehicle_control_command.launch.py
```

## Troubleshooting

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

### There is no data on the /joy topic

Ensure that `/dev/input/js0` is available from within ade.

If it is not available, restart `ade`, ensuring that it is run with the `--privileged` flag, or by
appropriately mounting the given device.

### The vehicle still does not move

First, ensure the whole stack is running properly, and is appropriately configured.

Next, ensure there is data on the `/joy` topic. If this is not the case, refer to the appropriate
question.

Next, check that data arrives on each of the following topics:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 topic hz /joy
ade$ ros2 topic hz /raw_command
ade$ ros2 topic hz /vehicle_cmd
```

If data is not arriving on one of these topics, then the stack did not start up correctly, or there
is a configuration problem. Stop the vehicle interface and joystick controller and inspect the logs
to determine which is the case.

If data is available on all topics, and the vehicle is still not moving, ensure that the port
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
