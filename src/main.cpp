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
#include "gtsam_graph.hpp"
#include "scan_matcher.hpp"
#include "load_ros_data.hpp"
#include "kdtreetype.hpp"
#include "conversions.hpp"
#include "pcl_filters.hpp"
#include "measurementtypes.hpp"

const uint64_t bias_offset = 1000000;
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

void outputParams(boost::shared_ptr<Params> p_)
{
  LOG_INFO("Outputting all parameters:");
  std::cout << "lidar_topic_map: " << p_->lidar_topic_map << std::endl;
  std::cout << "lidar_topic_loc: " << p_->lidar_topic_loc << std::endl;
  std::cout << "gps_topic: " << p_->gps_topic << std::endl;
  std::cout << "gps_imu_topic: " << p_->gps_imu_topic << std::endl;
  std::cout << "odom_topic: " << p_->odom_topic << std::endl;
  std::cout << "init_method: " << p_->init_method << std::endl;
  std::cout << "mapping_method: " << p_->mapping_method << std::endl;
  std::cout << "int_map_size: " << p_->int_map_size << std::endl;
  std::cout << "gps_type: " << p_->gps_type << std::endl;
  std::cout << "k_nearest_neighbours: " << p_->knn << std::endl;
  std::cout << "trajectory_sampling_distance: " << p_->trajectory_sampling_dist << std::endl;
  std::cout << "distance_match_min: " << p_->distance_match_min << std::endl;
  std::cout << "distance_match_limit: " << p_->distance_match_limit << std::endl;
  std::cout << "x_lower_threshold: " << p_->x_lower_threshold << std::endl;
  std::cout << "x_upper_threshold: " << p_->x_upper_threshold << std::endl;
  std::cout << "y_lower_threshold: " << p_->y_lower_threshold << std::endl;
  std::cout << "y_upper_threshold: " << p_->y_upper_threshold << std::endl;
  std::cout << "z_lower_threshold: " << p_->z_lower_threshold << std::endl;
  std::cout << "z_upper_threshold: " << p_->z_upper_threshold << std::endl;
  std::cout << "use_pass_through_filter: " << p_->use_pass_through_filter << std::endl;
  std::cout << "x_lower_threshold_map: " << p_->x_lower_threshold_map << std::endl;
  std::cout << "x_upper_threshold_map: " << p_->x_upper_threshold_map << std::endl;
  std::cout << "y_lower_threshold_map: " << p_->y_lower_threshold_map << std::endl;
  std::cout << "y_upper_threshold_map: " << p_->y_upper_threshold_map << std::endl;
  std::cout << "z_lower_threshold_map: " << p_->z_lower_threshold_map << std::endl;
  std::cout << "z_upper_threshold_map: " << p_->z_upper_threshold_map << std::endl;
  std::cout << "use_pass_through_filter_map: " << p_->use_pass_through_filter_map << std::endl;
  std::cout << "downsample_input: " << p_->downsample_input << std::endl;
  std::cout << "input_downsample_size: " << p_->input_downsample_size << std::endl;
  std::cout << "use_rad_filter: " << p_->use_rad_filter << std::endl;
  std::cout << "set_min_neighbours: " << p_->set_min_neighbours << std::endl;
  std::cout << "set_search_radius: " << p_->set_search_radius << std::endl;
  std::cout << "matcher_config_path: " << p_->matcher_config << std::endl;
  std::cout << "ground_segment: " << p_->ground_segment << std::endl;
  std::cout << "use_gps: " << p_->use_gps << std::endl;
  std::cout << "downsample_cell_size: " << p_->downsample_cell_size << std::endl;
  std::cout << "iterations: " << p_->iterations << std::endl;
  std::cout << "visualize: " << p_->visualize << std::endl;
  std::cout << "step_matches: " << p_->step_matches << std::endl;
  std::cout << "optimize_gps_lidar: " << p_->optimize_gps_lidar << std::endl;
  std::cout << "fixed_scan_transform_cov: " << p_->fixed_scan_transform_cov << std::endl;
  std::cout << "scan_transform_cov: " << p_->scan_transform_cov << std::endl;
}

int main()
{
  // Load parameters from ig_graph_slam_config:
    Params p__;
    fillparams(p__);
    boost::shared_ptr<Params> p_;
    p_ = boost::shared_ptr<Params>(&p__);
    std::cout << "Input Bag File: " << p_->bag_file_path << std::endl;
    rosbag::Bag bag;
    boost::shared_ptr<ScanMatcher> scan_matcher;
    boost::shared_ptr<ROSBag> load_ros_data;
    ROSBag load_ros_data_;
    load_ros_data = boost::shared_ptr<ROSBag>(&load_ros_data_);
    //ros::Time time_limit_s = ros::TIME_MIN + ros::Duration(1521829591);

  // Init pointer to scan matcher based on type
    if (p_->matcher_type == "icp1")
    {
        scan_matcher = boost::make_shared<ICP1ScanMatcher>(p__);
    }
    else if (p_->matcher_type == "icp2")
    {
        LOG_ERROR("%s matcher type is not yet implemented. Coming soon.", p_->matcher_type);
        // TODO: implement the same scan matcher but with point to plane
        //scan_matcher = boost::make_shared<ICP2ScanMatcher>(p_);
        return -1;
    }
    else
    {
        LOG_ERROR("%s is not a valid matcher type. Change matcher type in ig_graph_slam_config.yaml", p_->matcher_type);
        return -1;
    }

  // Read bag file
    try
    {
        bag.open(p_->bag_file_path, rosbag::bagmode::Read);
    }
    catch (rosbag::BagException &ex)
    {
        LOG_ERROR("Bag exception : %s", ex.what());
    }
    p_->topics.push_back(p_->lidar_topic_loc);
    p_->topics.push_back(p_->lidar_topic_map);
    p_->topics.push_back(p_->gps_topic);
    p_->topics.push_back(p_->gps_imu_topic);
    p_->topics.push_back(p_->odom_topic);

  // iterate through bag file and save messages
    int bag_counter = 0;
    bool end_of_bag = false;
    bool start_of_bag = true;
    rosbag::View view(bag, rosbag::TopicQuery(p_->topics), ros::TIME_MIN, ros::TIME_MAX, true);
    int total_messages = view.size();
    if (total_messages == 0)
    {
      LOG_ERROR("No messages read. Check your topics in config file.");
      outputParams(p_);
    }
    // iterate through bag messages and save imu messages only
    if( !(p_->gps_imu_topic == ""))
    {
      LOG_INFO("Loading IMU messages...");
      for (auto iter = view.begin(); iter != view.end(); iter++)
      {
        bag_counter++;
        if (bag_counter == total_messages)
        {
          end_of_bag = true;
        }
        load_ros_data->loadIMUMessage(iter, end_of_bag, start_of_bag, p_->gps_imu_topic);
        start_of_bag = false;
      }
    }
    end_of_bag = false;
    bag_counter = 0;;

    // iterate through bag and save GPS and Scan messages
        //-> we need to do this after filling imu container because GPS relies
        //   on imu for rotation information
    for (auto iter = view.begin(); iter != view.end(); iter++)
    {
      bag_counter++;
      if (bag_counter == total_messages)
      {
        end_of_bag = true;
      }
      LOG_INFO("Loading message No. %d of %d", bag_counter, total_messages);
      load_ros_data->loadROSBagMessage(iter, end_of_bag, p_);
    }
    bag.close();
std::cout << "TEST4" << std::endl;
  // Select scans to store and savae their respective poses based on gps measurements
    scan_matcher->createPoseScanMap(load_ros_data);

  // Determine how many scans to register against for each pose and save
    scan_matcher->findAdjacentScans();

  // perform graph optimization:

    // init. variables:
    GTSAMGraph graph;
    bool no_GPS = true;
    int cnt_match = 0;
    TimePoint last_timestamp;
    bool correction_norm_valid;

    // build graph
    for (int outer_loops = 0; outer_loops < p_->iterations; outer_loops++)
    { // Iterate to update initial estimates and redo matches
      cnt_match = 0;

      // clear graph and results between each iteration
      graph.clear();
      graph.result.clear();

      if (p_->optimize_gps_lidar)
      { // optimize transform from lidar to gps
          // THIS IS NOT YET IMPLEMENTED
          // Cannot use Ben's code because the way our T_LIDAR_GPS is
          // measured is different.
      }

      for (uint64_t j = 0; j < scan_matcher->adjacency->size(); j++)
      { // iterate over all j scans
          LOG_INFO("Matching scan %d of %d", j+1, scan_matcher->adjacency->size());
          if (j == 0)
          {
              last_timestamp =
                load_ros_data->getLidarScanTimePoint(scan_matcher->pose_scan_map.at(j));
          }

          // iterate over all scans adjacent to scan j
          for (auto iter = scan_matcher->adjacency->at(j).begin();
               iter != scan_matcher->adjacency->at(j).end();
               iter++)
          {
              //LOG_INFO("Matching scan %d of %d, against agjacency %d of %d..", j+1, scan_matcher->adjacency->size(), *iter, *scan_matcher->adjacency->at(j).end());
              Eigen::Affine3d T_Liter_Lj;
              wave::Mat6 info;
              bool match_success;

              // Attempts to match the scans and checks the correction norm
              // between scan transformation and GPS
              match_success = scan_matcher->matchScans(*iter, j, T_Liter_Lj, info, correction_norm_valid, load_ros_data);
              if (!correction_norm_valid) // WHAT IS THIS??
              {
                  continue;
              }
              if (match_success) // create factor in graph is scan successful
              {
                  wave::Mat6 mgtsam; // Eigen::Matrix<double, 6, 6>
                  // this next bit is a major gotcha, whole thing blows up
                  // without it. This just rearranges the info matrix to the correct form
                    // block starting at 0,0 of size 3x3
                  mgtsam.block(0, 0, 3, 3) = info.block(3, 3, 3, 3);  // rotation
                  mgtsam.block(3, 3, 3, 3) = info.block(0, 0, 3, 3);  // translation
                  mgtsam.block(0, 3, 3, 3) = info.block(0, 3, 3, 3);  // off diagonal
                  mgtsam.block(3, 0, 3, 3) = info.block(3, 0, 3, 3);  // off diagonal
                  graph.addFactor(*iter, j, T_Liter_Lj, mgtsam);
                  LOG_INFO("Match no %d of %d", cnt_match, scan_matcher->total_matches);
                  cnt_match++;
              }
              else
              {
                  LOG_ERROR("Scan match failed");
              }

          }

          if (p_->use_gps)
          { // only if we want to use gps priors
            // TODO: Implement the gps priors.
            // Careful because currently GPS has no rotation info
            // See Ben's code
          }
          graph.addInitialPose(scan_matcher->init_pose.poses.at(j), j);
      }
      if (no_GPS)
      {
          LOG_INFO("Fixing first pose.");
          graph.fixFirstPose();
      }
      LOG_INFO("Done building graph.");
      graph.print();
      //std::cout << "Hit 'Enter' to continue" << std::endl;
      //std::cin.get(); // wait for user to hit next
      graph.optimize();

      // Loop through and get final alignment
      Eigen::Affine3d temp_trans, prev;
      for (uint64_t k = 0; k < graph.poses.size(); k++)
      {
          // get resulting transform for pose k
          temp_trans.matrix() = graph.result.at<gtsam::Pose3>(graph.poses.at(k)).matrix();

          // update initial pose estimate for next iteration (if needed)
          scan_matcher->init_pose.poses.at(graph.poses.at(k)) = temp_trans;

          // Add result to final pose measurement container
          scan_matcher->final_poses.emplace_back(load_ros_data->getLidarScanTimePoint(graph.poses.at(k)), 0, temp_trans);

          // this seems to be useless?
          prev = temp_trans;
      }

    }

    // build and output map
    scan_matcher->createAggregateMap(graph, load_ros_data);
    scan_matcher->outputAggregateMap(graph, load_ros_data);

}
