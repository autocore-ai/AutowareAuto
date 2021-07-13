Localization Demo using rosbag {#rosbag-localization-howto}
==============================

@tableofcontents

# Running the Localization Demo

1. Setup the Autoware environment following instructions in the [installation guide](@ref installation).

2. In the console, download and extract the rosbag into the adehome.
```{bash}
> cd ~/adehome
> curl https://autoware-auto.s3.us-east-2.amazonaws.com/rosbag2/rosbag2-astuff-1-lidar-only.tar.gz | tar xz
```

3. Launch ADE
```{bash}
> cd ~/adehome/AutowareAuto
> ade --rc .aderc-lgsvl start --update --enter
```

4. In the same terminal launch the demo.
   ```{bash}
   > source /opt/AutowareAuto/setup.bash
   > ros2 launch autoware_demos localization_rosbag.launch.py
   ```
   Alternatively, the script can be launch with the `-s` flag to show more options for launching.
   ```{bash}
   > ros2 launch -s autoware_demos localization_rosbag.launch.py
   ```

5. The localization demo launch file uses the `load_initial_pose_from_parameters` parameter to set the initial pose in the NDT localization node. The vehicle should drive to the parking spot and then proceed to park. Once parked, the vehicle should exit the parking spot then proceed back towards the start point.

@image html images/localization-rosbag-initialization-location.png "Rosbag Initialization Point" width=600px
