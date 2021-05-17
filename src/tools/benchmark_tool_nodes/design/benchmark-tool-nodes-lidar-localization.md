lidar_localization_benchmark {#benchmark-tool-nodes-lidar-localization}
=============================

# Instruction to run the lidar_localization_benchmark

The lidar_localization_benchmark is using recorded lidar data from the AS
parking lot.

To run the benchmark, make sure to have the rosbag2 that can be retrieved from
the following link downloaded to a folder accessible by the node:

- <https://autoware-auto.s3.us-east-2.amazonaws.com/rosbag2/rosbag2-astuff-1-lidar-only.tar.gz>

The parking lot map also needs to be downloaded (instruction below).

Run these steps on the console (adjust it to your path):

``` bash
cd ~
wget https://autoware-auto.s3.us-east-2.amazonaws.com/rosbag2/rosbag2-astuff-1-lidar-only.tar.gz
tar -xzf rosbag2-astuff-1-lidar-only.tar.gz
rm rosbag2-astuff-1-lidar-only.tar.gz
# Now open the Autoware.auto repository
cd AutowareAuto
git lfs fetch --all
git lfs checkout .
```

After these commands you should see the ```rosbag2_2020_09_23-15_58_07``` file.

Now to run the benchmark just do:

``` bash
ros2 launch benchmark_tool_nodes lidar_localization_benchmark.launch.py
```

By default the node expects the location of the rosbag data to be in ```${HOME}```.
The ```rosbag_file_path``` parameter can be used to specify a different path.
