# Keyence LJ-V driver

This package provides a node that implements a minimal ROS driver for the
Keyence LJ-V (7000) Ultra-High Speed In-Line Profilometers.


## Supported devices

In theory all sensor heads in the LJ-V 7000 product range should be supported,
but only the V7080 has been tested.


## Sensor configuration

As this is not a full fledged driver, the sensor must be configured (with LJ
Navigator, under Windows) as follows before use with the driver node:

 1. Continuous Trigger mode, sampling frequency greater than or equal to that
    configured in the launch file
 1. No compression (neither x, nor time)
 1. Full width profiles (800 points)


## Installation

Clone from source, then build in a catkin workspace. Make sure to check that
all dependencies have been installed (using `rosdep`).


## Use

### Driver node
Start the driver either directly with the supplied `driver.launch` file, or
include `driver.launch` in another launch file. In both cases the
`controller_ip` argument should be provided. All other arguments have default
values (see `driver.launch` for more information).

After starting the driver, it will immediately connect to the LJ-V controller
at the configured IP and PORT combination. Only after the `profiles` topic has
been subscribed to will the driver start communicating with the controller
however.

All profiles are converted to PCL Point Clouds, published to the configured TF
frame (by default: `sensor_optical_frame`), which is assumed to exist (the
driver does not publish any TF frames itself). To correctly place profiles in
space, the `sensor_optical_frame` should have its origin located at the
'Reference distance (A)' from the sensor aperture (see the LJ-V Series Setup
Guide, page 2 for more information). The `keyence_ljv_description` package
provides a URDF for the LJ-V7080, where the required frames have already been
defined.

### Coordinate frames

Both the Keyence and ROS use a right-handed coordinate system. However, for the
sensor, X+ is pointing left, Y+ is back and Z+ is towards the sensor aperture.
Positive values in the profile are therefore closer to the sensor, negative
values are further away from it.

### Visualisation of profiles

Visualisation of Point Clouds can be done using the `PointCloud2` visualisation
Display type in RViz.

Set `Style` to `Points`, and select the appropriate topic. As the published
profiles are small (maximum of 40 mm across), make sure RViz is zoomed in
sufficiently.

### Dynamic reconfigure

The driver node has support for dynamic reconfiguration of some of its
parameters. In particular the `scale_factor` and `cnv_inf_pts` parameters, which
control the amount of scaling that is performed on profiles before publication,
and whether profile points reported as having 'infinite distance' by the Keyence
sensor should have their Z coordinates set to positive infinity (as per
REP-117), respectively.

By default, no scaling is performed, and the driver honours REP-117.
