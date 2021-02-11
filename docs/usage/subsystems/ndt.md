Initializing the NDT localizer {#ndt-initialization}
==============================

@tableofcontents

The LGSVL simulation always spawns the car at a particular location of the map that is different
from the origin. The NDT localizer used in the Autoware.Auto stack requires an initial guess of the
vehicle pose that is somewhere close to the truth. From this guess, NDT finds the best pose (i.e.
localizes) in an interactive optimization. Once initialized, NDT uses the information of the previous
time step to initialize and does not need further user input. It should localize as the car moves
around.

# Prerequisites

-# Follow the @ref lgsvl instructions.
-# Start the Autoware.Auto stack including the NDT component and the RViz visualization; e.g. as per the [AVP demo instructions](@ref avpdemo-simulation-launch).

The following error message is expected in the terminal in which the stack was started until the localization is successful:

    [localization.p2d_ndt_localizer_node]: Could not find a connection between 'map' and 'base_link' because they are not part of the same tree.Tf has two or more unconnected trees.

There are two ways to send an initial guess of the vehicle pose to the system:

1. Select pose graphically in RViz.
2. Publish a pose in the terminal.

Option 1. is more flexible, as one can drive the car around in the simulation to any spot on the available map, and then choose to initialize the localization there.

Option 2. is easier to achieve reproducible results.

Initially RViz is centered on the origin of the map frame but the vehicle is spawned to a different
location in the map, still unknown to RViz. Zoom in/out with the mouse wheel and pan the view with
the middle-mouse button in RViz. Press `F1` in the simulator window to see the camera controls. The
following figure shows the vehicle at its spawning point and RViz zoomed out enough to display the
origin of the map frame and the location of the vehicle. The origin of the map is indicated with a
small white grid containing a white outline of the supposed (but incorrect) vehicle pose.

@image html images/avp_before_localization.png "Initial views of simulator and RViz" width=90%

# Initializing with RViz {#ndt-initialization-rviz}

Check that in RViz, the `map` fixed frame is selected. Make sure the map and TF visualizations are
enabled and you have a good top-down view.

In RViz, click on `2D pose estimation` button and then click/drag on the position of the vehicle such that the arrow points in the same direction as the car and release the mouse button. A message with the pose estimate is then sent to NDT.

Here is what `rviz` should look like when setting the initial pose:
@image html images/avp-rviz-init.png "Initialize localization with rviz" width=70%

A `tf` visualization (the red-blue-green axes) should appear at that location and the `Could not find a connection` error messages in the terminal should stop.

Here is what `rviz` should look like after successful initialization, depending on which visualizations are enabled; the important ones are the colorful `tf` crosses for `odom` and `base_link` whose marker scale has been increased for better visibility within RViz:

@image html images/avp_after_localization.png "Zoom-in on vehicle in RViz after localization initialization" width=70%

@note Upon first localization, there is a jump and the RViz display needs to be re-centered to show the vehicle.

If the initial pose estimate was close, you should be able to drive around in the simulator and see the `base_link` frame track the car's position

@warning The localization may not be able to follow if the car is manually driven in a jerky manner, or if the CPU load is too and some lidar packets are dropped. If the localization is stuck, restart the stack and re-initialize.

Note that the pose can be echoed and copied for later use with the second initialization method:

```
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 topic echo /localization/initialpose
```

# Initializing with a predefined pose {#ndt-initialization-predefined}

In order to initialize localization at the spawning point of the simulation as defined in @ref lgsvl, enter the following command in a new terminal:

```
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 topic pub --once /localization/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
{header : {
    stamp : {
        sec: 0,
        nanosec: 0
    },
    frame_id : "map"
},
pose : {
    pose : {
        position : {
            x: -57.463,
            y: -41.644,
            z: -2.01,
        },
        orientation : {
            x: 0.0,
            y: 0.0,
            z: -0.99917,
            w: 0.04059,
        },
    },
    covariance : [
        0.25, 0.0,  0.0, 0.0, 0.0, 0.0,
        0.0,  0.25, 0.0, 0.0, 0.0, 0.0,
        0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
        0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
        0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
        0.0,  0.0,  0.0, 0.0, 0.0, 0.068,
    ],
}}
"
```
