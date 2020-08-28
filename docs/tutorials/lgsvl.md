LGSVL simulator {#lgsvl}
========

[TOC]

# LGSVL simulator: running the LGSVL simulator alongside Autoware.Auto

The following guide assumes that the LGSVL simulator will be run from inside an ADE container.

For more information about the simulator, see:

[https://www.lgsvlsimulator.com/docs/](https://www.lgsvlsimulator.com/docs/)

## Requirements

- ADE 4.2.0 or later. Follow the
[ADE installation instructions](https://ade-cli.readthedocs.io/en/latest/install.html) to install it
- NVidia graphics card
- If using Docker engine version 19.03 or later, [install Native GPU Support](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(Native-GPU-Support)).
- If using Docker engine with a version less than 19.03, either upgrade Docker or [install nvidia-docker2](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0)).

## Using the simulator

Using the simulator involves the following steps:

-# Launch it
-# Configure a vehicle
-# Choose or create a simulation
-# Bridge the simulator with Autoware.Auto
-# Start the simulations

This section outlines these steps.

### Launching the simulator

Install ADE as described in the
[installation section](installation-and-development.html#installation-and-development-install-ade):

Start ADE with the LGSVL volume:

```
$ cd ~/adehome/AutowareAuto
$ ade --rc .aderc-lgsvl start --update --enter
```

To start the LGSVL simulator, in the same terminal window:

```
ade$ source /opt/AutowareAuto/setup.bash
ade$ /opt/lgsvl/simulator
```

Now start your favorite browser and go to [http://127.0.0.1:8080](http://127.0.0.1:8080) where simulations can be configured.

**Note:** Since CycloneDDS is now the default DDS implementation in `ade`, prefixing `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to the simulator command is no longer necessary.
However, if you are not using the latest version of the Autoware.Auto Docker containers or source code, you may receive errors when launching LGSVL.
If this is the case, try prefixing the command above with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

**Note:** When running LGSVL Simulator in a Docker container, the "Open Browser..." button in the simulator window does not work.

**Note:** When running LGSVL Simulator for the first time, you may be asked to log into [https://account.lgsvlsimulator.com/](https://account.lgsvlsimulator.com/).
If you have an account, log in. If you do not have an account, create one, then log in.

### Configuring a vehicle

To configure the Lexus model, do the following in the browser:

- In the Vehicles tab look for `Lexus2016RXHybrid`. If not available, follow [these instructions](https://www.lgsvlsimulator.com/docs/vehicles-tab/#how-to-add-a-vehicle)
to add it and use the URL `https://assets.lgsvlsimulator.com/ea5e32fe566065c6d1bbf1f0728d6654c94e375d/vehicle_AWFLexus2016RXHybrid`

  - Click on the wrench icon for the Lexus vehicle
  - Change the bridge type to `ROS2 Native`
  - In the `Sensors:` box, copy and paste the content of the `lgsvl-sensors.json` file in the root of the AutowareAuto repository

The above steps are a modified version of the
[LGSVL documentation](https://www.lgsvlsimulator.com/docs/autoware-auto-instructions/#run-simulator-alongside-autowareauto)

### Choosing/creating a simulation

Choose `Simulations` on the left to see the simulations screen. The LGSVL simulator lets you store and reuse multiple simulation configurations. To use an existing simulation, select the desired simulation and press the play button in the bottom right corner of the screen. The simulator should now start in the LGSVL window.

To create a new simulation, follow the below steps:

- Switch to the Simulations tab and click the `Add new` button
- Enter a name and switch to the `Map & Vehicles` tab
- Select a map from the drop down menu. If none are available follow [this guide](https://www.lgsvlsimulator.com/docs/maps-tab/#where-to-find-maps) to get a map.
- Select the `Lexus2016RXHybrid` from the drop down menu. In the bridge connection box to the right enter the bridge address. For the default setting, use `127.0.0.1:9090`.
- Click submit

Once the simulation has been created, you can select and run it.

### Using the ROS2 Native Bridge

The "ROS2 Native Bridge" is a special bridge type which does not require a websocket-based bridge.
When a simulation is started, the topics should be published in ROS 2 automatically.

### Bridging with Autoware.Auto

LGSVL uses conventions which are not directly aligned with ROS 2 conventions. The full list of behaviors the `lgsvl_interface` implements is:
-# Converts control inputs with CCW positive rotations to the CCW negative inputs the LGSVL
simulator expects
-# Provides a mapping from `VehicleControlCommand` to the `RawControlCommand` LGSVL expects via
parametrizable 1D lookup tables

To run the `lgsvl_interface`, enter the following in a new terminal window:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run lgsvl_interface lgsvl_interface_exe __params:=/opt/AutowareAuto/share/lgsvl_interface/param/lgsvl.param.yaml
```

Launch scripts are also provided for convenience. For example for a joystick control demo, run the following in a new terminal window:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch lgsvl_interface lgsvl_joystick.launch.py
```

For an example of using `VehicleControlCommand` with LGSVL, run the following demo in a new terminal window:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch lgsvl_interface lgsvl_vehicle_control_command.launch.py
```

## Troubleshooting

### You don't see the "ROS2 Native" bridge option

Your version of the `ade` container for LGSVL is probably out-of-date.
Update it using the following commands, once you have left `ade` with `exit`:

```
ade stop
git pull
source .aderc-lgsvl
ade start --update --enter
```

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

Update the `src/tools/joystick_vehicle_interface/param/logitech_f310.defaults.param.yaml` appropriately, or
make a copy.

### There are no data on the /joy topic

Ensure that `/dev/input/js0` is available from within ade.

**Note:**  Sourcing the `.aderc-lgsvl` file should achieve this through the `ADE_DOCKER_RUN_ARGS` environment variable.

If it is not available, restart `ade`, ensuring that the device is appropriately mounted. Alternatively, restart `ade` and run it with the `--privileged` flag, e.g.:

```
$ ade start <ade arguments> -- --privileged
```

### The vehicle still does not move

First, ensure the whole stack is running properly, and is appropriately configured. See the section
above titled "No data are being sent through to ROS."

Next, ensure there are data on the `/joy` topic. If this is not the case, refer to the appropriate
question.
