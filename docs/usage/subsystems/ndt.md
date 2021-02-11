Initializing the NDT localizer {#ndt-howto}
==============================

[TOC]

# Starting localization

* Start LGSVL with @ref lgsvl
* Start `ms3_sim.launch.py`
* In RViz, select the `map` fixed frame
Make sure the map and TF visualizations are enabled and you have a good top-down view
* Start the LGSVL simulation
The following error message is expected:

  [localization.p2d_ndt_localizer_node]: Could not find a connection between 'map' and 'base_link' because they are not part of the same tree.Tf has two or more unconnected trees.

* In RViz, click on "2D pose estimation" and then click/drag on the position of the vehicle such that the arrow points in the same direction as the car
* A `tf` visualization (the red-blue-green axes) should appear at that location and the "Could not find a connection" error messages in the terminal should stop
* If the initial pose estimate was close, you should be able to drive around in the simulator and see the `base_link` frame track the car's position

Here is what `rviz` should look like before setting the initial pose:
![RViz before localization initialization](images/avp_before_localization.png)

Here is what `rviz` should look like after, depending on which visualizations are enabled (the important ones are the colorful `tf` crosses for `odom` and `base_link`):
![RViz after localization initialization](images/avp_after_localization.png)
