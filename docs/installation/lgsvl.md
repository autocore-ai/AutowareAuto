LGSVL simulator {#lgsvl}
========

@tableofcontents

# LGSVL simulator: running the LGSVL simulator alongside Autoware.Auto

LGSVL is a Unity-based multi-robot simulator for autonomous vehicle developers. It provides a simulated world to

- create sensor inputs to Autoware.Auto,
- allow the user to manually steer the ego vehicle similar to a computer game,
- place other moving traffic participants in a scene.

For more information about the simulator, see [https://www.lgsvlsimulator.com/docs/](https://www.lgsvlsimulator.com/docs/).

# Requirements

The following guide assumes that the LGSVL simulator will be run from inside an ADE container, although it is not strictly required.

- ADE 4.2.0 or later. Follow the
[ADE installation instructions](https://ade-cli.readthedocs.io/en/latest/install.html) to install it
- NVidia graphics card
- If using Docker engine version 19.03 or later, [install Native GPU Support](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(Native-GPU-Support)).
- If using Docker engine with a version less than 19.03, either upgrade Docker or [install nvidia-docker2](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0))
- Cyclone DDS is the DDS vendor; see @ref choosing-a-dds-vendor

# Using the simulator

Using the simulator involves the following steps:

-# Launch it
-# Choose or create a simulation
-# Bridge the simulator with Autoware.Auto
-# Start the simulation

This section outlines these steps.

## Launching the simulator

Install ADE as described in the [installation section](@ref installation-ade):

Start ADE with the LGSVL volume:

```{bash}
$ cd ~/adehome/AutowareAuto
$ ade --rc .aderc-lgsvl start --update --enter
```

Pick a different `.aderc-*-lgsvl` file to select a non-default ROS version.

To start the LGSVL simulator, in the same terminal window:

```{bash}
ade$ /opt/lgsvl/simulator
```

Now start your favorite browser on the host system (outside of ADE!) and go to [http://127.0.0.1:8080](http://127.0.0.1:8080) where simulations can be configured.

@note When running LGSVL Simulator in a Docker container, the "Open Browser..." button in the simulator window does not work.

@note When running LGSVL Simulator for the first time, you may be asked to log into [https://account.lgsvlsimulator.com/](https://account.lgsvlsimulator.com/).
If you have an account, log in. If you do not have an account, create one, then log in.

### Troubleshooting

In case the simulator window opens up with a black screen and the application immediately terminates, have a look at the log file at

```{bash}
~/.config/unity3d/LG\ Silicon\ Valley\ Lab/LGSVL\ Simulator/Player.log
```

One possible fix is to remove conflicting graphics drivers from ADE with

```{bash}
ade$ sudo apt remove mesa-vulkan-drivers
```
and launch the simulator again.

## Creating a simulation

Creating a simulation configuration takes only a few clicks in the browser. The following steps assume that the launch was successful and illustrate the configuration process with the setup for the @ref avpdemo.

### Choosing a map

The goal is to create a map configuration for the AutonomouStuff parking lot. If that map is already available on the first launch of the simulation, nothings needs to be done.

Else follow the [LGSVL instructions](https://www.lgsvlsimulator.com/docs/maps-tab/#where-to-find-maps), click the `Add new` button and enter a name (e.g. `AutonomouStuff parking lot`) and the link to the asset bundle from [this site](https://content.lgsvlsimulator.com/maps/autonomoustuff/) containing the map data:

`https://assets.lgsvlsimulator.com/ec057870762b5a967a451c93444b67d0b64e9656/environment_AutonomouStuff`

Once submitted, this will download the map automatically.

@image html images/lgsvl-map.png "Choosing a map"

### Configuring a vehicle {#lgsvl-configuring-vehicle}
The goal is to create a vehicle configuration for the AutonomouStuff parking lot.

Follow the [LGSVL instructions](https://www.lgsvlsimulator.com/docs/vehicles-tab/#how-to-add-a-vehicle),
to configure the Lexus model: click the vehicles tab, then `Add new` and enter
`Lexus2016RXHybrid` as name and

`https://assets.lgsvlsimulator.com/ea5e32fe566065c6d1bbf1f0728d6654c94e375d/vehicle_AWFLexus2016RXHybrid`

as `Vehicle URL`.

@image html images/lgsvl-vehicle.png "Adding a vehicle"

Once submitted, click on the wrench icon for the Lexus vehicle and

- Change the bridge type to `Ros2NativeBridge`
- In the `Sensors` box, copy and paste the content of  [`avp-sensors.json`](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/launch/autoware_demos/config/svl/avp-sensors.json) located in the Autoware.Auto repository at `src/tools/autoware_demos/config/svl` to tell LGSVL about sensor positions and where to communicate information to the Autoware.Auto stack.

@image html images/lgsvl-bridge-sensors.png "Configuring bridge and sensors"

The `Ros2NativeBridge` is a special bridge type which does not require a websocket-based bridge.
When a simulation is started, the topics should be published in ROS 2 automatically.

The above steps are a modified version of the
[LGSVL documentation](https://www.lgsvlsimulator.com/docs/autoware-auto-instructions/#run-simulator-alongside-autowareauto)

### Choosing/creating a simulation

Choose `Simulations` on the left to see the simulations screen. The LGSVL simulator lets you store and reuse multiple simulation configurations. To use an existing simulation, select the desired simulation and press the play button in the bottom right corner of the screen. The simulator should now start in the LGSVL window.

To create a new simulation, follow the below steps:

- Switch to the Simulations tab and click the `Add new` button.
- Enter a name and switch to the `Map & Vehicles` tab.
- Select the `Lexus2016RXHybrid` from the drop-down menu.
- Enter `127.0.0.1:9090` in the `Ros2NativeBridge connction` box.
- No changes to the `Traffic` or `Weather` tab are needed but one can play around here.
- Click submit.

@image html images/lgsvl-simulation-general.png "Configuring the simulation"
@image html images/lgsvl-simulation-map-and-vehicle.png "Configuring the simulation map and vehicle"

### Starting the simulation {#lgsvl-start-simulation}

Once the simulation has been created, select it by clicking on its white box first, then run it by clicking the play button.

@image html images/lgsvl-simulation-start.png "Starting the simulation" width=80%

The Lexus should appear in a 3D rendering in the `LGSVL Simulator` window (not in the browser).

The next step is to control the Lexus and to drive around. Press `F1` to see a list of shortcuts and press the cookie button in bottom left corner for more UI controls.

The essential commands are to use the arrow keys to steer and accelerate, and the `Page Up` and `Page Down` keys to switch between forward and reverse driving.

Congratulations if everything is working up to this point. The setup of LGSVL is completed.

@image html images/lgsvl-controls.png "Controlling the Lexus"

@todo #850 Uncomment joystick session when tested again

<!-- ### Controlling LGSVL with a joystick -->

<!-- It is possible to control the simulation with a gamepad or joystick instead of a keyboard. Assuming just one joystick is plugged into the system, just map it into the Docker container when starting ADE by appending the proper `--device` flag: -->

<!-- ``` -->
<!-- $ ade start <ade arguments> -- --device /dev/input/js0 -->
<!-- ``` -->

<!-- @note The instructions in this section were tested with a Logitech Gamepad F310 -->

<!-- #### Troubleshooting -->

<!-- ### The brake/throttle/steering does not work -->

<!-- The joystick control mapping is not deterministic. It is occasionally necessary to modify the axis -->
<!-- mapping. -->

<!-- First, with the joystick controller running, verify that you can see the raw messages by running -->
<!-- the following: -->

<!-- ``` -->
<!-- $ ade enter -->
<!-- ade$ source /opt/AutowareAuto/setup.bash -->
<!-- ade$ ros2 topic echo /joy -->
<!-- ``` -->

<!-- Next, actuate the appropriate axis on the vehicle controllers to determine which buttons and joy -->
<!-- sticks correspond to which indices in the `Joy` message. -->

<!-- Update the `src/tools/joystick_vehicle_interface/param/logitech_f310.defaults.param.yaml` appropriately, or -->
<!-- make a copy. -->

<!-- ### There are no data on the /joy topic -->

<!-- Ensure that `/dev/input/js0` is available from within ADE. -->

<!-- @todo Specific instructions. What should a user do exactly? -->

<!-- If it is not available, restart `ade`, ensuring that the device is appropriately mounted. Alternatively, restart `ade` and run it with the `--privileged` flag, e.g.: -->

<!-- ``` -->
<!-- $ ade start <ade arguments> -- --privileged -->
<!-- ``` -->

<!-- ### The vehicle still does not move -->

<!-- First, ensure the whole stack is running properly, and is appropriately configured. See the section -->
<!-- above titled "No data are being sent through to ROS." -->

<!-- Next, ensure there are data on the `/joy` topic. If this is not the case, refer to the appropriate -->
<!-- question. -->

# Bridging with Autoware.Auto

@todo update check section

LGSVL uses conventions which are not directly aligned with ROS 2 conventions. The full list of behaviors the `lgsvl_interface` implements is:
-# Converts control inputs with CCW positive rotations to the CCW negative inputs the LGSVL
simulator expects
-# Provides a mapping from `VehicleControlCommand` to the `RawControlCommand` LGSVL expects via
parametrizable 1D lookup tables

To run the `lgsvl_interface` manually, enter the following in a new terminal window:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 run lgsvl_interface lgsvl_interface_exe --ros-args --params-file /opt/AutowareAuto/share/lgsvl_interface/param/lgsvl.param.yaml
```

Autoware.Auto uses PointCloud2 messages with `x,y,z,intensity` rather than `x,y,z,intensity,timestamp` fields.

This node will convert `points_xyzi`

Run `point_type_adapter` to convert the messages.

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch point_type_adapter point_type_adapter.launch.py
```

Launch scripts are also provided for convenience. For example for a joystick control demo, run the following in a new terminal window:

```{bash}
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch joystick_vehicle_interface_nodes lgsvl_joystick.launch.py
```

For an example of using `VehicleControlCommand` with LGSVL, run the following demo in a new terminal window:

```{bash}
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch lgsvl_interface lgsvl_vehicle_control_command.launch.py
```
