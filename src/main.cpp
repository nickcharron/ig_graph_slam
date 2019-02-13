// Basic Headers
#include <Eigen/Core>
#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>

// ROS Headers
#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// PCL Headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// WAVE Headers
#include <wave/matching/icp.hpp>
#include <wave/utils/log.hpp>

// IG Graph SLAM Headers
#include "conversions.hpp"
#include "gtsam_graph.hpp"
#include "kdtreetype.hpp"
#include "load_ros_data.hpp"
#include "measurementtypes.hpp"
#include "pcl_filters.hpp"
#include "scan_matcher.hpp"
#include "utils.hpp"

const uint64_t bias_offset = 1000000;
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

int main() {

  // create shared pointers for parameters and fill the params
  boost::shared_ptr<Params> p_(new Params);
  fillparams(*p_);
  if (!validateParams(p_)) {
    LOG_INFO("NOTE: Check all booleans for typos in config file.");
    outputParams(p_);
    return 0;
  }

  // create shared pointers for ros data and initialize
  boost::shared_ptr<ROSBag> load_ros_data;
  load_ros_data = boost::make_shared<ROSBag>(p_->bag_file_path, p_->topics);

  // create shared pointer to scan matcher based on type, and initialize
  boost::shared_ptr<ScanMatcher> scan_matcher;
  std::string matcherConfigPath = getMatcherConfig(p_->matcher_type);
  LOG_INFO("Loading matcher config file: %s", matcherConfigPath.c_str());

  if (p_->matcher_type == "icp") {
    scan_matcher = boost::make_shared<ICPScanMatcher>(*p_, matcherConfigPath);
  } else if (p_->matcher_type == "loam") {
    LOG_ERROR("%s matcher type is not yet implemented. Coming soon.",
              p_->matcher_type.c_str());
    // TODO: implement loam scan matcher
    return -1;
  } else if (p_->matcher_type == "gicp") {
    scan_matcher = boost::make_shared<GICPScanMatcher>(*p_, matcherConfigPath);
  } else {
    LOG_ERROR("%s is not a valid matcher type. Change matcher type in "
              "ig_graph_slam_config.yaml",
              p_->matcher_type.c_str());
    return -1;
  }

  // iterate through bag messages and save imu messages only
  load_ros_data->loadIMUMessagesAll(p_->imu_topic);

  // iterate through bag and save GPS and Scan messages
  // NOTE: we need to do this after filling imu container because GPS relies
  // on imu for rotation information
  load_ros_data->loadROSBagMessagesAll(p_);

  // Select scans to store and save their respective poses based on
  // initialization measurements
  scan_matcher->createPoseScanMap(load_ros_data);

  // perform graph optimization:

  // init. variables:
  GTSAMGraph graph;
  bool no_GPS = true;
  int cnt_match = 0;
  TimePoint last_timestamp;
  bool correction_norm_valid;

  // build graph
  for (int outer_loops = 0; outer_loops < p_->iterations;
       outer_loops++) { // Iterate to update initial estimates and redo matches
    LOG_INFO("Iteration No. %d of %d.", outer_loops + 1, p_->iterations);
    cnt_match = 0;
    graph.clear();
    graph.result.clear();

    // Determine how many scans to register against for each pose and save
    scan_matcher->findAdjacentScans();

    // Determine how many scans to register against for each pose and save
    scan_matcher->findLoops();

    if (p_->optimize_gps_lidar) { // optimize transform from lidar to gps
                                  // TODO: THIS IS NOT YET IMPLEMENTED
                                  // Cannot use Ben's code because the way our
                                  // T_LIDAR_GPS is measured is different.
    }

    for (uint64_t j = 0; j < scan_matcher->adjacency->size();
         j++) { // iterate over all j scans
      if (j == 0) {
        last_timestamp = load_ros_data->getLidarScanTimePoint(
            scan_matcher->pose_scan_map.at(j));
      }

      // iterate over all scans adjacent to scan j
      // for (auto iter = scan_matcher->adjacency->at(j).begin();
      //      iter != scan_matcher->adjacency->at(j).end(); iter++) {
      for (uint64_t iter = 0; iter < scan_matcher->adjacency->at(j).size();
           iter++) {
        Eigen::Affine3d T_Liter_Lj;
        wave::Mat6 info;
        bool match_success = false;

        // Attempts to match the scans and checks the correction norm
        // between scan transformation and GPS
        // TODO: this is a hack fix, bug in adjacency builder.
        if (scan_matcher->adjacency->at(j).at(iter) >=
            scan_matcher->init_pose.poses.size()) {
          LOG_ERROR("Trying to access pose that doesn't exist");
        } else {
          match_success = scan_matcher->matchScans(
              scan_matcher->adjacency->at(j).at(iter), j, T_Liter_Lj, info,
              correction_norm_valid, load_ros_data);
        }

        if (!correction_norm_valid) // WHAT IS THIS??
        {
          continue;
        }
        if (match_success) // create factor in graph if scan successful
        {
          wave::Mat6 mgtsam; // Eigen::Matrix<double, 6, 6>
          // this next bit is a major gotcha, whole thing blows up
          // without it. This just rearranges the info matrix to the correct
          // form block starting at 0,0 of size 3x3
          mgtsam.block(0, 0, 3, 3) = info.block(3, 3, 3, 3); // rotation
          mgtsam.block(3, 3, 3, 3) = info.block(0, 0, 3, 3); // translation
          mgtsam.block(0, 3, 3, 3) = info.block(0, 3, 3, 3); // off diagonal
          mgtsam.block(3, 0, 3, 3) = info.block(3, 0, 3, 3); // off diagonal
          graph.addFactor(scan_matcher->adjacency->at(j).at(iter), j,
                          T_Liter_Lj, mgtsam);
          outputPercentComplete(cnt_match, scan_matcher->total_matches,
                                "Matching scans between nearest neighbours...");
          cnt_match++;
        } else {
          LOG_ERROR("Scan match failed");
        }
      }

      if (p_->use_gps) { // only if we want to use gps priors
                         // TODO: Implement the gps priors.
                         // Careful because currently GPS has no rotation info
                         // See Ben's code
      }
      graph.addInitialPose(scan_matcher->init_pose.poses.at(j), j);
    }

    for (uint64_t j = 0; j < scan_matcher->loops->size();
         j++) { // iterate over all j loop closures

      Eigen::Affine3d T_L1_L2;
      wave::Mat6 info;
      bool match_success;

      uint64_t L1 = scan_matcher->loops->at(j)[0];
      uint64_t L2 = scan_matcher->loops->at(j)[1];

      match_success = scan_matcher->matchScans(
          L1, L2, T_L1_L2, info, correction_norm_valid, load_ros_data);

      if (!correction_norm_valid) // WHAT IS THIS??
      {
        continue;
      }

      if (match_success) // create factor in graph if scan successful
      {
        wave::Mat6 mgtsam; // Eigen::Matrix<double, 6, 6>
        // this next bit is a major gotcha, whole thing blows up
        // without it. This just rearranges the info matrix to the correct form
        // block starting at 0,0 of size 3x3
        mgtsam.block(0, 0, 3, 3) = info.block(3, 3, 3, 3); // rotation
        mgtsam.block(3, 3, 3, 3) = info.block(0, 0, 3, 3); // translation
        mgtsam.block(0, 3, 3, 3) = info.block(0, 3, 3, 3); // off diagonal
        mgtsam.block(3, 0, 3, 3) = info.block(3, 0, 3, 3); // off diagonal
        graph.addFactor(L1, L2, T_L1_L2, mgtsam);
        outputPercentComplete(cnt_match, scan_matcher->total_matches,
                              "Matching scans between nearest neighbours...");
        cnt_match++;
      } else {
        LOG_ERROR("Scan match failed");
      }
    }

    if (no_GPS) {
      LOG_INFO("Fixing first pose.");
      graph.fixFirstPose();
    }
    LOG_INFO("Done building graph.");
    graph.optimize();

    // Loop through and get final alignment
    LOG_INFO("Updating Poses for Next Iteration.");
    Eigen::Affine3d temp_trans, prev;
    for (uint64_t k = 0; k < graph.poses.size(); k++) {
      // get resulting transform for pose k
      temp_trans.matrix() =
          graph.result.at<gtsam::Pose3>(graph.poses.at(k)).matrix();

      // update initial pose estimate for next iteration (if needed)
      scan_matcher->init_pose.poses.at(graph.poses.at(k)) = temp_trans;

      // Add result to final pose measurement container
      scan_matcher->final_poses.emplace_back(
          load_ros_data->getLidarScanTimePoint(graph.poses.at(k)), 0,
          temp_trans);

      // this seems to be useless?
      prev = temp_trans;
    }
  }

  // Check output directory
  std::string save_path = p_->output_path + "/" +
                          convertTimeToDate(std::chrono::system_clock::now()) +
                          "/";
  boost::filesystem::create_directories(save_path);

  // Save yaml file
  std::string dateandtime = convertTimeToDate(std::chrono::system_clock::now());
  std::string dstFileName = save_path + dateandtime + "_params.txt";
  std::string yamlDirStr = __FILE__;
  yamlDirStr.erase(yamlDirStr.end() - 20, yamlDirStr.end());
  yamlDirStr += "config/ig_graph_slam_config.yaml";
  std::ifstream src(yamlDirStr, std::ios::binary);
  std::ofstream dst(dstFileName, std::ios::binary);
  dst << src.rdbuf();

  // Save Graph file
  std::string graphFileName = save_path + dateandtime + "gtsam_graph.dot";
  graph.print(graphFileName, false);

  // build and output maps
  scan_matcher->createAggregateMap(graph, load_ros_data, 1);
  scan_matcher->outputAggregateMap(graph, load_ros_data, 1, save_path);
  scan_matcher->createAggregateMap(graph, load_ros_data, 2);
  scan_matcher->outputAggregateMap(graph, load_ros_data, 2, save_path);
  scan_matcher->createAggregateMap(graph, load_ros_data, 3);
  scan_matcher->outputAggregateMap(graph, load_ros_data, 3, save_path);
}
