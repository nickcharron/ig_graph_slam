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
bash ./scripts/install_deps.sh
```

5. Install xdot for viewing gtsam graph file outputted after mapping

```
sudo apt install xdot
```

## Additional Information and Useful Resources

GTSAM discussion group: https://groups.google.com/forum/#!forum/gtsam-users

GTSAM concepts: https://bitbucket.org/gtborg/gtsam/src/90f688d94c720ac859689e62d6a003dabc79bc42/GTSAM-Concepts.md?fileviewer=file-view-default

GTSAM primer: https://bitbucket.org/gtborg/gtsam/wiki/Home

GTSAM extra install instructions: https://bitbucket.org/gtborg/gtsam/src/90f688d94c720ac859689e62d6a003dabc79bc42/INSTALL?fileviewer=file-view-default
