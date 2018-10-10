// Basic Headers
#include <boost/filesystem.hpp>
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


const uint64_t bias_offset = 1000000;
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

void outputParams(Params p_)
{
  std::cout << "lidar_topic: " << p_.lidar_topic << std::endl;
  std::cout << "gps_topic: " << p_.gps_topic << std::endl;
  std::cout << "k_nearest_neighbours: " << p_.knn << std::endl;
  std::cout << "trajectory_sampling_distance: " << p_.trajectory_sampling_dist << std::endl;
  std::cout << "max_range: " << p_.max_range << std::endl;
  std::cout << "x_lower_threshold: " << p_.x_lower_threshold << std::endl;
  std::cout << "x_upper_threshold: " << p_.x_upper_threshold << std::endl;
  std::cout << "y_lower_threshold: " << p_.y_lower_threshold << std::endl;
  std::cout << "y_upper_threshold: " << p_.y_upper_threshold << std::endl;
  std::cout << "z_lower_threshold: " << p_.z_lower_threshold << std::endl;
  std::cout << "z_upper_threshold: " << p_.z_upper_threshold << std::endl;
  std::cout << "use_pass_through_filter: " << p_.use_pass_through_filter << std::endl;
  std::cout << "downsample_input: " << p_.downsample_input << std::endl;
  std::cout << "input_downsample_size: " << p_.input_downsample_size << std::endl;
  std::cout << "use_rad_filter: " << p_.use_rad_filter << std::endl;
  std::cout << "set_min_neighbours: " << p_.set_min_neighbours << std::endl;
  std::cout << "set_search_radius: " << p_.set_search_radius << std::endl;
  std::cout << "matcher_config_path: " << p_.matcher_config << std::endl;
  std::cout << "ground_segment: " << p_.ground_segment << std::endl;
  std::cout << "use_gps: " << p_.use_gps << std::endl;
  std::cout << "downsample_cell_size: " << p_.downsample_cell_size << std::endl;
  std::cout << "iterations: " << p_.iterations << std::endl;
  std::cout << "visualize: " << p_.visualize << std::endl;
  std::cout << "step_matches: " << p_.step_matches << std::endl;
  std::cout << "optimize_gps_lidar: " << p_.optimize_gps_lidar << std::endl;
  std::cout << "fixed_scan_transform_cov: " << p_.fixed_scan_transform_cov << std::endl;
  std::cout << "scan_transform_cov: " << p_.scan_transform_cov << std::endl;
}

int main()
{
  // Load parameters from ig_graph_slam_config:
    Params p_;
    fillparams(p_);
    std::cout << "Input Bag File: " << p_.bag_file_path <<std::endl;
    //outputParams(p_);
    rosbag::Bag bag;
    boost::shared_ptr<ScanMatcher> scan_matcher;
    //ros::Time time_limit_s = ros::TIME_MIN + ros::Duration(1521829591);


  // Init pointer to scan matcher based on type
    if (p_.matcher_type == "icp1")
    {
        scan_matcher = boost::make_shared<ICP1ScanMatcher>(p_);
    }
    else if (p_.matcher_type == "icp2")
    {
        LOG_ERROR("%s matcher type is not yet implemented. Coming soon.", p_.matcher_type);
        // TODO: implement the same scan matcher but with point to plane
        //scan_matcher = boost::make_shared<ICP2ScanMatcher>(p_);
        return -1;
    }
    else
    {
        LOG_ERROR("%s is not a valid matcher type. Change matcher type in ig_graph_slam_config.yaml", p_.matcher_type);
        return -1;
    }

  // Read bag file
    try
    {
        bag.open(p_.bag_file_path, rosbag::bagmode::Read);
    }
    catch (rosbag::BagException &ex)
    {
        LOG_ERROR("Bag exception : %s", ex.what());
    }
    p_.topics.push_back(p_.lidar_topic);
    p_.topics.push_back(p_.gps_topic);

  // iterate through bag file and save messages
    int bag_counter = 0;
    bool end_of_bag = false;
    rosbag::View view(bag, rosbag::TopicQuery(p_.topics), ros::TIME_MIN, ros::TIME_MAX, true);
    int total_messages = view.size();
    for (auto iter = view.begin(); iter != view.end(); iter++)
    {
      bag_counter++;
      if (bag_counter == total_messages)
      {
        end_of_bag = true;
      }
      LOG_INFO("Loading message No. %d of %d", bag_counter, total_messages);
      scan_matcher->loadROSBagMessage(iter, end_of_bag);
    }
    bag.close();
}
