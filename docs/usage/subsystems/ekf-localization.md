Running the EKF filter for localization {#ekf-localization-howto}
=======================================

[TOC]

# Ndt EKF smooth localization

This demo aims to show Kalman filter smoothing on top of NDT localization. The pose messages from
NDT get a covariance assigned, then get passed into the Kalman filter node. The smooth output is
then published as an Odometry topic and visualized in rviz2. This demo is tested with the Lexus car
in LG simulator on the parking lot map.

Before running the demo, ensure that ADE is running. If not, it can be started as in the example below:

```console
$ cd ~/adehome/AutowareAuto
$ ade start --update --enter
```

Then, run the demo as follows:

```console
$ ade start
ade$ ros2 launch autoware_demos ekf_ndt_smoothing_lgsvl.launch.py
```
