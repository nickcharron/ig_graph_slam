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

bagdir="/home/$u/mapping/loam.bag"
gsconfig="/home/$u/mapping/ig_graph_slam_config.yaml"
posedir="/home/$u/mapping/poses.json"
mapbuilderconfig="/home/$u/mapping/map_builder_config.json"
cd ~/
echo "Starting ig_graph_slam with bag:"
echo "$bagdir"
echo "and config file: "
echo "$posedir"
./catkin_ws/build/ig_graph_slam/ig_graph_slam_main $gsconfig $posedir

echo "Starting map builder with pose file: "
echo "$posedir"
echo "and config file: "
echo "$mapbuilderconfig"
./catkin_ws/build/libbeam/beam_mapping/beam_mapping_main $mapbuilderconfig
