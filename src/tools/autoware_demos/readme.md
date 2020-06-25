Autoware demos
==============

# Ndt EKF smooth localization

This demo aims to show Kalman filter smoothing on top of NDT localizaition. The pose messages from
NDT get a covariance assigned, then get passed into the Kalman filter node. The smooth output is
then published as an Odometry topic and visualized in rviz2. This demo is tested with the Lexus car
in LG simluator on the parking lot map.

Run the demo as follows:
```bash
ros2 launch autoware_demos ekf_ndt_smoothing_lgsvl.launch.py
```
