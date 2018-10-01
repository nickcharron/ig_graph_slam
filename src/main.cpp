// Basic Headers
//#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <limits>
#include <Eigen/Core>

// ROS Headers
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
// #include <novatel_msgs/INSPVAX.h

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

// WAVE Headers
#include <wave/containers/measurement_container.hpp>
#include <wave/utils/math.hpp>
#include <wave/utils/config.hpp>
#include <wave/utils/log.hpp>
#include <wave/matching/icp.hpp>
#include <wave/matching/gicp.hpp>
#include <wave/matching/pointcloud_display.hpp>
#include <wave/matching/ground_segmentation.hpp>

// IG Graph SLAM Headers
#include <gtsam_graph.hpp>
#include "scan_matcher.hpp"
//#include "kdtreetype.hpp"


// const uint64_t bias_offset = 1000000;
// using Clock = std::chrono::steady_clock;
// using TimePoint = std::chrono::time_point<Clock>;

int main()
{
  // Load parameters from ig_graph_slam_config:
    Params p_;
    fillparams(p_);
    std::cout << "Params File: " << std::endl;
    std::cout << "Bag: " << p_.bag_file_path <<std::endl;
    std::cout << "Optimize Lidar Transform: " << p_.optimize_gps_lidar << std::endl;







}
