#!/bin/bash
set -e

u="$USER"

echo "Collecting bag file for loam initialized graph slam."
echo "Saving to: "
echo "/home/$u/loam.bag"
rosbag record -O "/home/$u/mapping/loam.bag" \
/clock \
/vvlp/velodyne_points \
/hvlp/velodyne_points \
/ig/loam/lidar_odom \
/tf \
/tf_static \
/clock \
__name:=gs_loam_bag &
