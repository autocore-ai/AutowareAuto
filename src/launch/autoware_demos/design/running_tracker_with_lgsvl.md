Running multi object tracker with the lgsvl simulator {#running-tracker-with-vision}
===============================

# Setup the simulator
The object tracker in autoware is designed to subscribe to lidar cluster based objects and 2d detections from vision. To setup the simulator to do this change the sensor configuration of the vehicle. Go to the "Vehicles" page, select the desired vehicle and click the wrench icon on it. Copy and paste the contents from [here](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/perception/segmentation/ground_truth_detections/config/lgsvl-sensors-camera.json) into the box. Also the `ROS2` bridge should be picked from the drop-down list. Create a simulation that uses this modified vehicle and a map of your choice. The current demo is designed to work on the AutonomouStuff parking lot map.

# Install lgsvl-bridge
The sensor configuration mentioned above uses some sensors that are not supported by the native bridge. So to use the configuration lgsvl-bridge needs to be installed and run. This will convert the messages from the simulator into ros2 messages that autoware can understand. Inside the ade run,
```
sudo apt update && sudo apt install ros-foxy-lgsvl-bridge
```

# Running the tracker
Hit the play button on the browser window to start the simulation that was created at the beginning.  
From a terminal inside ade run the command,  `lgsvl_bridge`  
Run the following launch command to launch the tracker, 
```
ros2 launch autoware_demos lidar_tracks_lgsvl.launch.py
```

This command by default uses the NDT localizer to get the ego vehicle state. But the NDT loclaizer requires a point cloud map which is available only for the `AutonomouStuff` map of the simulator. If you want to run the tracker in some other simulator map (for example, `BorregasAve`) run the following command to disable NDT and use ego vehicle state from the `lgsvl_interface`,
```
ros2 launch autoware_demos lidar_tracks_lgsvl.launch.py use_ndt:=False
```


