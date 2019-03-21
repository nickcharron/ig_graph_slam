#!/bin/bash
set -e

u="$USER"

echo "Stopping bag."
rosnode kill /gs_loam_bag
echo "Stopping loam_velodyne."
rosnode kill /ig/loam/multiScanRegistration
rosnode kill /ig/loam/laserMapping
rosnode kill /ig/loam/transformMaintenance
rosnode kill /ig/loam/laserOdometry

savedir="/home/$u/OPG_DEMO_JAN2019/pcds"
if [ -d $savedir ]; then
  rm -rf $savedir
fi
mkdir -p $savedir

bagdir="/home/$u/OPG_DEMO_JAN2019/bags/ig_scan_loam_OPG_JAN2019.bag"
# gsdir="$(rospack find ig_graph_slam)"
echo "Starting ig_graph_slam with bag:"
echo "$bagdir"
echo "Saving maps to: "
echo "$savedir"

cd $savedir
../../ig_catkin_ws/build/ig_graph_slam/ig_graph_slam_node
# $gsdir/../../build/ig_graph_slam/ig_graph_slam_node
