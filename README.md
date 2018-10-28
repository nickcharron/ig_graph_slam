# Package Overview
This package is made to perform graph SLAM for the Inspector Gadget using GTSAM.
This is still a work in progress. We are currently using GPS and Lidar, but plan
on incorporating IMU, wheel odometry, and visual odometry.

## Installation
Note: This package assumes you have install the package inspector_gadget
(https://github.com/nickcharron/inspector_gadget) and all its dependencies
before installing ig_graph_slam.

1. Install novatel span driver

		Note: this is only needed to work with novatel gps data, however you will
		need it to build the ig_graph_slam node (for now)

		TODO: Make two branches, one for Moose and one for IG so we don't need to
		use this novatel span driver

		```
		cd /path/to/catkin_ws/src
		git clone https://github.com/nickcharron/novatel_span_driver.git
		```

2. Clone this repo to your catkin workspace:

```
cd /path/to/catkin_ws/src
git clone https://github.com/nickcharron/ig_graph_slam.git
```

3. Download and install Intel MKL from https://software.intel.com/en-us/mkl

	 Note: this is not required, but having it yields better performance with gtsam

4. Install all dependencies:

```
cd /ig_graph_slam
bash ./scritps/install_deps.sh
```
**NOTE:** For now, the interpolation function needs to be disabled in the measurement
containers used from libwave. To do this:
a) open /path_to/libwave/wave_containers/include/wave/containers/impl/measurement_container.hpp
b) comment out line 162 (return interpolate(std::prev(i_next), i_next, t);)
c) add return i_next->value; in its place

**TODO:** Add an overload function to redefine the interpolation function. See
this example in the anm_localizer:
 	https://github.com/wavelab/autonomoose/blob/master/rospackages/autonomoose_core/anm_localizer/include/anm_localizer/utils/measurement_types.hpp#L115
	https://github.com/wavelab/autonomoose/blob/master/rospackages/autonomoose_core/anm_localizer/include/anm_localizer/anm_localizer.hpp#L309

## Additional Information and Useful Resources

GTSAM discussion group: https://groups.google.com/forum/#!forum/gtsam-users

GTSAM concepts: https://bitbucket.org/gtborg/gtsam/src/90f688d94c720ac859689e62d6a003dabc79bc42/GTSAM-Concepts.md?fileviewer=file-view-default

GTSAM primer: https://bitbucket.org/gtborg/gtsam/wiki/Home

GTSAM extra install instructions: https://bitbucket.org/gtborg/gtsam/src/90f688d94c720ac859689e62d6a003dabc79bc42/INSTALL?fileviewer=file-view-default

## TO DO LIST
- filter map scans
- add calibrations
- add T_base_lidar for horizontal localization
- add ability to localize off vertical scans
- move all functions for filling measurement containers to new file  
- add interpolation overload function for transforms
- change output directory for maps 
