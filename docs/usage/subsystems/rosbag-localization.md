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
