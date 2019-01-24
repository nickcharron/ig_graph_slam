#!/bin/bash
set -e

u="$USER"
savedir="/home/$u/OPG_DEMO_JAN2019/bags"

if [ -d $savedir ]; then
  rm -rf $savedir
fi
mkdir -p $savedir

echo "Starting loam_velodyne..."
roslaunch loam_velodyne ig_loam_OPG_DEMO.launch &

echo "Collecting bag file for loam initialized graph slam."
echo "Saving to: "
echo "$savedir/ig_scan_loam_OPG_JAN2019.bag"
rosbag record -O $savedir/ig_scan_loam_OPG_JAN2019.bag \
/clock \
/vvlp/velodyne_points \
/hvlp/velodyne_points \
/ig/loam/lidar_odom \
__name:=gs_loam_bag
