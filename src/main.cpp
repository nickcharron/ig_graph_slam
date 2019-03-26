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

// libbeam specific headers
#include <beam/utils/math.hpp>

// IG Graph SLAM Headers
#include "conversions.hpp"
#include "gtsam_graph.hpp"
#include "load_ros_data.hpp"
#include "measurementtypes.hpp"
#include "pcl_filters.hpp"
#include "scan_matcher.hpp"
#include "utils.hpp"

const uint64_t bias_offset = 1000000;
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

int main() {

  // Get start time:
  std::chrono::system_clock::time_point time_start =
      std::chrono::system_clock::now();

  // create shared pointers for parameters and fill the params
  boost::shared_ptr<Params> p_(new Params);
  p_ = boost::make_shared<Params>();
  // p_->fillparams(*p_);
  if (!p_->validateParams()) {
    LOG_INFO("NOTE: Check all booleans for typos in config file.");
    p_->outputParams();
    return 0;
  }

  // create shared pointers for ros data and initialize
  boost::shared_ptr<ROSBag> load_ros_data;
  load_ros_data = boost::make_shared<ROSBag>(*p_);
  LOG_INFO("Loading config file: %s", p_->config_file_path.c_str());
  LOG_INFO("Input Bag File: %s", p_->bag_file_path.c_str());

  // create shared pointer to scan matcher based on type, and initialize
  boost::shared_ptr<ScanMatcher> scan_matcher;
  std::string matcherConfigPath = p_->getMatcherConfig();
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
  load_ros_data->loadIMUMessagesAll();

  // iterate through bag and save GPS and Scan messages
  // NOTE: we need to do this after filling imu container because GPS relies
  // on imu for rotation information
  load_ros_data->loadROSBagMessagesAll();

  // Output duration to load all ros messages
  outputTimePointDiff(time_start, std::chrono::system_clock::now(),
                      "Time to load ROS data: ");

  // Check output directory and create
  std::string save_path = p_->output_path + "/" +
                          convertTimeToDate(std::chrono::system_clock::now()) +
                          "/";
  boost::filesystem::create_directories(save_path);

  // Save yaml file
  scan_matcher->saveParamsFile(save_path);

  GTSAMGraph graph;
  if (p_->use_prev_poses) {
    // load poses from file
    scan_matcher->loadPrevPoses();

    // create pose scan map
    scan_matcher->createPoseScanMapFromPoses(load_ros_data);
  } else {
    // Select scans to store and save their respective poses based on
    // initialization measurements
    scan_matcher->createPoseScanMap(load_ros_data);

    // Ouput initial trajectory
    scan_matcher->outputInitTraj(save_path);

    // perform graph optimization:
    graph = scan_matcher->buildGTSAMGraph(load_ros_data);

    // Save Graph file
    scan_matcher->saveGraphFile(graph, save_path);
  }

  // build and output maps
  scan_matcher->createAggregateMap(load_ros_data, 1);
  scan_matcher->outputAggregateMap(1, save_path);
  scan_matcher->outputOptTraj(save_path);
  scan_matcher->createAggregateMap(load_ros_data, 2);
  scan_matcher->outputAggregateMap(2, save_path);
  // Uncomment if you want to combine the two maps and save them
  // scan_matcher->createAggregateMap(load_ros_data, 3);
  // scan_matcher->outputAggregateMap(3, save_path);

  outputTimePointDiff(time_start, std::chrono::system_clock::now(),
                      "Total computation time: ");
}
