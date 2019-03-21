#!/bin/bash
set -e

u="$USER"

echo "input absolute path to bag/save directory: (e.g., /home/nick/bag_files)"
read savedir
#savedir="/home/$u/bag_files/ig_scans/2018_02_14_OPG_Darlington"

echo "input bag file name: (e.g., bag1)"
read bagname

echo "Enter start offset for bag playbag: (e.g., 35)"
read startoffset

echo "Enter playback rate for ros bag: (e.g., 0.5)"
read playbackrate

echo "Starting loam_velodyne..."
roslaunch loam_velodyne ig_loam_norespawn.launch &

sleep 2

echo "Collecting bag file for loam initialized graph slam."
echo "Saving to: "
echo "${savedir}/${bagname}_loam.bag"
rosbag record -O "${savedir}/${bagname}_loam.bag" \
/clock \
/vvlp/velodyne_points \
/hvlp/velodyne_points \
/ig/loam/lidar_odom \
/tf \
/tf_static \
/clock \
__name:=gs_loam_bag &


echo "playing ros bag $bagname.bag"
rosbag play -s $startoffset -r $playbackrate "${savedir}/${bagname}.bag" --clock

echo "Done."
echo "killing all nodes..."
rosnode kill -a

echo "killing rosmaster..."
killall -9 rosmaster
