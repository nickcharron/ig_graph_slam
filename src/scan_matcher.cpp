// ROS and other Headers
#include <boost/filesystem.hpp>
#include <fstream>
#include <math.h>
#include <sstream>
#include <string>
#include <unistd.h>

#include <chrono>
#include <ctime>

// PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// WAVE Headers
#include "wave/matching/gicp.hpp"
#include <wave/matching/icp.hpp>
#include <wave/matching/pointcloud_display.hpp>
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

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

// General Functions

void fillparams(Params &params) {
  wave::ConfigParser parser;
  parser.addParam("gps_type", &(params.gps_type));
  parser.addParam("gps_topic", &(params.gps_topic));
  parser.addParam("imu_topic", &(params.imu_topic));
  parser.addParam("odom_topic", &(params.odom_topic));
  parser.addParam("init_method", &(params.init_method));
  parser.addParam("lidar_topic_loc", &(params.lidar_topic_loc));
  parser.addParam("lidar_topic_map", &(params.lidar_topic_map));
  parser.addParam("bag_file_path", &(params.bag_file_path));
  parser.addParam("use_prev_poses", &(params.use_prev_poses));
  parser.addParam("prev_poses_path", &(params.prev_poses_path));
  parser.addParam("T_LIDAR_GPS", &(params.T_LIDAR_GPS.matrix()));
  parser.addParam("T_LMAP_LLOC", &(params.T_LMAP_LLOC.matrix()));
  parser.addParam("use_pass_through_filter", &(params.use_pass_through_filter));
  parser.addParam("x_upper_threshold", &(params.x_upper_threshold));
  parser.addParam("x_lower_threshold", &(params.x_lower_threshold));
  parser.addParam("y_upper_threshold", &(params.y_upper_threshold));
  parser.addParam("y_lower_threshold", &(params.y_lower_threshold));
  parser.addParam("z_upper_threshold", &(params.z_upper_threshold));
  parser.addParam("z_lower_threshold", &(params.z_lower_threshold));
  parser.addParam("downsample_input", &(params.downsample_input));
  parser.addParam("input_downsample_size", &(params.input_downsample_size));
  parser.addParam("use_rad_filter", &(params.use_rad_filter));
  parser.addParam("set_min_neighbours", &(params.set_min_neighbours));
  parser.addParam("set_search_radius", &(params.set_search_radius));
  parser.addParam("ground_segment", &(params.ground_segment));
  parser.addParam("downsample_output_method",
                  &(params.downsample_output_method));
  parser.addParam("downsample_cell_size", &(params.downsample_cell_size));
  parser.addParam("int_map_size", &(params.int_map_size));
  parser.addParam("use_pass_through_filter_map",
                  &(params.use_pass_through_filter_map));
  parser.addParam("x_upper_threshold_map", &(params.x_upper_threshold_map));
  parser.addParam("x_lower_threshold_map", &(params.x_lower_threshold_map));
  parser.addParam("y_upper_threshold_map", &(params.y_upper_threshold_map));
  parser.addParam("y_lower_threshold_map", &(params.y_lower_threshold_map));
  parser.addParam("z_upper_threshold_map", &(params.z_upper_threshold));
  parser.addParam("z_lower_threshold_map", &(params.z_lower_threshold_map));
  parser.addParam("k_nearest_neighbours", &(params.knn));
  parser.addParam("trajectory_sampling_distance",
                  &(params.trajectory_sampling_dist));
  parser.addParam("map_sampling_distance", &(params.map_sampling_dist));
  parser.addParam("trajectory_rotation_change",
                  &(params.trajectory_rotation_change));
  parser.addParam("map_rotation_change", &(params.map_rotation_change));
  parser.addParam("distance_match_min", &(params.distance_match_min));
  parser.addParam("distance_match_limit", &(params.distance_match_limit));
  parser.addParam("loop_max_distance", &(params.loop_max_distance));
  parser.addParam("loop_min_travel_distance",
                  &(params.loop_min_travel_distance));
  parser.addParam("iterations", &(params.iterations));
  parser.addParam("use_gps", &(params.use_gps));
  parser.addParam("optimize_gps_lidar", &(params.optimize_gps_lidar));
  parser.addParam("fixed_scan_transform_cov",
                  &(params.fixed_scan_transform_cov));
  parser.addParam("scan_transform_cov", &(params.scan_transform_cov));
  parser.addParam("visualize", &(params.visualize));
  parser.addParam("step_matches", &(params.step_matches));
  parser.addParam("combine_scans", &(params.combine_scans));
  parser.addParam("matcher_type", &(params.matcher_type));
  parser.addParam("output_path", &(params.output_path));

  std::string yamlDirStr = __FILE__;
  yamlDirStr.erase(yamlDirStr.end() - 20, yamlDirStr.end());
  yamlDirStr += "config/ig_graph_slam_config.yaml";
  ifstream fileName(yamlDirStr.c_str());

  if (fileName.good()) {
    LOG_INFO("Loading config file: %s", yamlDirStr.c_str());
    parser.load(yamlDirStr);
    LOG_INFO("Input Bag File: %s", params.bag_file_path.c_str());
  } else {
    LOG_ERROR("ig_graph_slam.yaml not found in config folder");
  }

  params.topics.push_back(params.lidar_topic_loc);
  params.topics.push_back(params.lidar_topic_map);
  params.topics.push_back(params.gps_topic);
  params.topics.push_back(params.imu_topic);
  params.topics.push_back(params.odom_topic);
}

void outputParams(boost::shared_ptr<Params> p_) {
  std::cout
      << "----------------------------" << std::endl
      << "Outputting all parameters:" << std::endl
      << "----------------------------" << std::endl
      << "gps_type: " << p_->gps_type << std::endl
      << "gps_topic: " << p_->gps_topic << std::endl
      << "imu_topic: " << p_->imu_topic << std::endl
      << "odom_topic: " << p_->odom_topic << std::endl
      << "init_method: " << p_->init_method << std::endl
      << "lidar_topic_loc: " << p_->lidar_topic_loc << std::endl
      << "lidar_topic_map: " << p_->lidar_topic_map << std::endl
      << "bag_file_path: " << p_->bag_file_path << std::endl
      << "use_prev_poses: " << p_->use_prev_poses << std::endl
      << "prev_poses_path: " << p_->prev_poses_path << std::endl
      << "T_LIDAR_GPS: " << p_->T_LIDAR_GPS.matrix() << std::endl
      << "T_LMAP_LLOC: " << p_->T_LMAP_LLOC.matrix() << std::endl
      << "use_pass_through_filter: " << p_->use_pass_through_filter << std::endl
      << "x_upper_threshold: " << p_->x_upper_threshold << std::endl
      << "x_lower_threshold: " << p_->x_lower_threshold << std::endl
      << "y_upper_threshold: " << p_->y_upper_threshold << std::endl
      << "y_lower_threshold: " << p_->y_lower_threshold << std::endl
      << "z_upper_threshold: " << p_->z_upper_threshold << std::endl
      << "z_lower_threshold: " << p_->z_lower_threshold << std::endl
      << "downsample_input: " << p_->downsample_input << std::endl
      << "input_downsample_size: " << p_->input_downsample_size << std::endl
      << "use_rad_filter: " << p_->use_rad_filter << std::endl
      << "set_min_neighbours: " << p_->set_min_neighbours << std::endl
      << "set_search_radius: " << p_->set_search_radius << std::endl
      << "ground_segment: " << p_->ground_segment << std::endl
      << "downsample_output_method: " << p_->downsample_output_method
      << std::endl
      << "downsample_cell_size: " << p_->downsample_cell_size << std::endl
      << "int_map_size: " << p_->int_map_size << std::endl
      << "use_pass_through_filter_map: " << p_->use_pass_through_filter_map
      << std::endl
      << "x_upper_threshold_map: " << p_->x_upper_threshold_map << std::endl
      << "x_lower_threshold_map: " << p_->x_lower_threshold_map << std::endl
      << "y_upper_threshold_map: " << p_->y_upper_threshold_map << std::endl
      << "y_lower_threshold_map: " << p_->y_lower_threshold_map << std::endl
      << "z_upper_threshold_map: " << p_->z_upper_threshold_map << std::endl
      << "z_lower_threshold_map: " << p_->z_lower_threshold_map << std::endl
      << "k_nearest_neighbours: " << p_->knn << std::endl
      << "trajectory_sampling_distance: " << p_->trajectory_sampling_dist
      << std::endl
      << "map_sampling_distance: " << p_->map_sampling_dist << std::endl
      << "distance_match_min: " << p_->distance_match_min << std::endl
      << "distance_match_limit: " << p_->distance_match_limit << std::endl
      << "loop_max_distance: " << p_->loop_max_distance << std::endl
      << "loop_min_travel_distance: " << p_->loop_min_travel_distance
      << std::endl
      << "iterations: " << p_->iterations << std::endl
      << "use_gps: " << p_->use_gps << std::endl
      << "optimize_gps_lidar: " << p_->optimize_gps_lidar << std::endl
      << "fixed_scan_transform_cov: " << p_->fixed_scan_transform_cov
      << std::endl
      << "visualize: " << p_->visualize << std::endl
      << "step_matches: " << p_->step_matches << std::endl
      << "combine_scans: " << p_->combine_scans << std::endl
      << "matcher_type: " << p_->matcher_type << std::endl
      << "output_path: " << p_->output_path << std::endl
      << "----------------------------" << std::endl;
}

std::string getMatcherConfig(std::string matcher_type_){
	std::string matcherConfigPath = __FILE__;
	matcherConfigPath.erase(matcherConfigPath.end() - 20, matcherConfigPath.end());
	if (matcher_type_ == "icp") {
	    matcherConfigPath += "config/icp.yaml";
	    return matcherConfigPath;
	} else if (matcher_type_ == "loam") {
	  LOG_ERROR("%s matcher type is not yet implemented. Coming soon.",
		      matcher_type_.c_str());
	  return "";
	} else if (matcher_type_ == "gicp") {
	  matcherConfigPath += "config/gicp.yaml";
          return matcherConfigPath;
        } else {
	  LOG_ERROR("%s is not a valid matcher type. Change matcher type in ig_graph_slam_config.yaml", matcher_type_.c_str());
	  return "";
        }
}

bool validateParams(boost::shared_ptr<Params> p_) {
  // NOTE: These checks need to be performed in the same order as the
  // fillparams function, or else you may not catch the first error.
  if (!(p_->gps_type == "NavSatFix" || p_->gps_type == "INSPVAX")) {
    if (p_->init_method == 1) {
      LOG_ERROR("Invalid GPS type. Supported: NavSatFix, INSPVAX");
      return 0;
    }
  }

  if (p_->gps_topic == "" && p_->init_method == 1) {
    LOG_ERROR("Please enter GPS topic.");
    return 0;
  }

  if (p_->imu_topic == "" && p_->init_method == 1 &&
      p_->gps_type == "NavSatFix") {
    LOG_ERROR("Please enter GPS/IMU topic.");
    return 0;
  }

  if (p_->odom_topic == "" && p_->init_method == 2) {
    LOG_ERROR("Please enter odometry topic.");
    return 0;
  }

  if (!(p_->init_method == 1 || p_->init_method == 2)) {
    LOG_ERROR("Invalid initialization method. Enter 1 or 2.");
    return 0;
  }

  if (p_->lidar_topic_loc == "") {
    LOG_ERROR("Please enter localization lidar topic.");
    return 0;
  }

  if (p_->lidar_topic_map == "") {
    LOG_INFO("WARNING: No lidar map topic.");
  }

  if (!boost::filesystem::exists(p_->bag_file_path)) {
    LOG_ERROR("Cannot find bag file.");
    return 0;
  }

  if (!(p_->use_prev_poses == 1 || p_->use_prev_poses == 0)) {
    LOG_ERROR("Invalid parameter: use_prev_poses. Enter a boolean.");
    return 0;
  }

  if (!boost::filesystem::exists(p_->prev_poses_path) &&
      p_->use_prev_poses == 1) {
    LOG_ERROR("Cannot find previous poses file.");
    return 0;
  }

  if (!isTransformationMatrix(p_->T_LIDAR_GPS.matrix())) {
    LOG_ERROR("Invalid transformation matrix: T_LIDAR_GPS. Did not pass "
              "transformation check.");
    return 0;
  }

  if (!isTransformationMatrix(p_->T_LMAP_LLOC.matrix())) {
    LOG_ERROR("Invalid transformation matrix: T_LMAP_LLOC. Did not pass "
              "transformation check.");
    return 0;
  }

  if (!(p_->use_pass_through_filter == 1 || p_->use_pass_through_filter == 0)) {
    LOG_ERROR("Invalid parameter: use_pass_through_filter. Enter a boolean.");
    return 0;
  }

  if (!(p_->downsample_input == 1 || p_->downsample_input == 0)) {
    LOG_ERROR("Invalid parameter: downsample_input. Enter a boolean.");
    return 0;
  }

  if (!(p_->use_rad_filter == 1 || p_->use_rad_filter == 0)) {
    LOG_ERROR("Invalid parameter: downsample_input. Enter a boolean.");
    return 0;
  }

  if (p_->set_min_neighbours == 0) {
    LOG_ERROR("Invalid parameter: set_min_neighbours. Enter an integer greater "
              "than 0.");
    return 0;
  }

  if (!(p_->ground_segment == 1 || p_->ground_segment == 0)) {
    LOG_ERROR("Invalid parameter: ground_segment. Enter a boolean.");
    return 0;
  }

  if (!(p_->downsample_output_method == 1 ||
        p_->downsample_output_method == 2 ||
        p_->downsample_output_method == 3)) {
    LOG_ERROR("Invalid parameter: downsample_output_method. Enter 1, 2, or 3.");
    return 0;
  }

  if (p_->int_map_size == 0) {
    LOG_ERROR(
        "Invalid parameter: int_map_size. Enter an integer greater than 0.");
    return 0;
  }

  if (!(p_->use_pass_through_filter_map == 1 ||
        p_->use_pass_through_filter_map == 0)) {
    LOG_ERROR(
        "Invalid parameter: use_pass_through_filter_map. Enter a boolean.");
    return 0;
  }

  if (p_->x_upper_threshold < p_->x_lower_threshold ||
      p_->y_upper_threshold < p_->y_lower_threshold ||
      p_->z_upper_threshold < p_->z_lower_threshold ||
      p_->x_upper_threshold_map < p_->x_lower_threshold_map ||
      p_->y_upper_threshold_map < p_->y_lower_threshold_map ||
      p_->z_upper_threshold_map < p_->z_lower_threshold_map) {
    LOG_ERROR("Check pass through filter parameters. Limits not correct.");
    return 0;
  }

  if (p_->knn == 0) {
    LOG_ERROR("Invalid parameter: k_nearest_neighbours. Enter an integer "
              "greater than 0.");
    return 0;
  }

  if (p_->loop_min_travel_distance < p_->distance_match_limit) {
    LOG_ERROR("Parameter loop_min_travel_distance must be greater than "
              "parameter distance_match_limit");
    return 0;
  }

  if (p_->trajectory_sampling_dist < p_->map_sampling_dist) {
    LOG_ERROR("parameter trajectory_sampling_distance must be greater or equal "
              "to parameter map_sampling_distance");
    return 0;
  }

  if (p_->distance_match_limit < p_->distance_match_min) {
    LOG_ERROR("parameter distance_match_limit must be greater than parameter "
              "distance_match_min");
    return 0;
  }

  if (p_->trajectory_rotation_change < 5) {
    LOG_ERROR(
        "parameter trajectory_rotation_change invalid. Might be too small");
    return 0;
  }

  if (p_->map_rotation_change < 3) {
    LOG_ERROR("parameter map_rotation_change invalid. Might be too small");
    return 0;
  }

  if (p_->iterations == 0) {
    LOG_ERROR(
        "Invalid parameter: iterations. Enter an integer greater than 0.");
    return 0;
  }

  if (!(p_->use_gps == 1 || p_->use_gps == 0)) {
    LOG_ERROR("Invalid parameter: use_gps. Enter a boolean.");
    return 0;
  }

  if (!(p_->optimize_gps_lidar == 1 || p_->optimize_gps_lidar == 0)) {
    LOG_ERROR("Invalid parameter: optimize_gps_lidar. Enter a boolean.");
    return 0;
  }

  if (!(p_->fixed_scan_transform_cov == 1 ||
        p_->fixed_scan_transform_cov == 0)) {
    LOG_ERROR("Invalid parameter: fixed_scan_transform_cov. Enter a boolean.");
    return 0;
  }

  if (!(p_->visualize == 1 || p_->visualize == 0)) {
    LOG_ERROR("Invalid parameter: visualize. Enter a boolean.");
    return 0;
  }

  if (!(p_->step_matches == 1 || p_->step_matches == 0)) {
    LOG_ERROR("Invalid parameter: step_matches. Enter a boolean.");
    return 0;
  }

  if (!(p_->combine_scans == 1 || p_->combine_scans == 0)) {
    LOG_ERROR("Invalid parameter: combine_scans. Enter a boolean.");
    return 0;
  }

  if (!(p_->matcher_type == "icp" || p_->matcher_type == "gicp" ||
        p_->matcher_type == "loam")) {
    if (p_->matcher_type == "loam") {
      LOG_ERROR("LOAM Matcher not yet implemented");
    } else {
      LOG_ERROR("Invalid parameter: matcher_type. Supported: icp, gicp, loam");
    }
    return 0;
  }

  std::string matcherConfigFilePath = getMatcherConfig(p_->matcher_type);

  if (!boost::filesystem::exists(matcherConfigFilePath)) {
    if (p_->matcher_type == "icp") {
      LOG_ERROR("icp.yaml not found. Looking in: %s", matcherConfigFilePath.c_str());
    } else if (p_->matcher_type == "gicp") {
      LOG_ERROR("gicp.yaml not found. Looking in: %s", matcherConfigFilePath.c_str());
    } else if (p_->matcher_type == "loam") {
      LOG_ERROR("loam.yaml not found. Looking in: %s", matcherConfigFilePath.c_str());
    } else {
      LOG_ERROR("matcher config file not found. Looking in: %s", matcherConfigFilePath.c_str());
    }
    return 0;
  }
  return 1;
}

// General Functions
bool takeNewScan(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2,
                 const double &dist, const double &rot) {
  // calculate the norm of the distance between the two points
  double l2sqrd = (p1(0, 3) - p2(0, 3)) * (p1(0, 3) - p2(0, 3)) +
                  (p1(1, 3) - p2(1, 3)) * (p1(1, 3) - p2(1, 3)) +
                  (p1(2, 3) - p2(2, 3)) * (p1(2, 3) - p2(2, 3));

  double minRotSq = rot * DEG_TO_RAD * rot * DEG_TO_RAD;
  Eigen::Vector3d eps1, eps2, diffSq;
  eps1 = RToLieAlgebra(p1.rotation());
  eps2 = RToLieAlgebra(p2.rotation());
  diffSq(0, 0) = (eps2(0, 0) - eps1(0, 0)) * (eps2(0, 0) - eps1(0, 0));
  diffSq(1, 0) = (eps2(1, 0) - eps1(1, 0)) * (eps2(1, 0) - eps1(1, 0));
  diffSq(2, 0) = (eps2(2, 0) - eps1(2, 0)) * (eps2(2, 0) - eps1(2, 0));

  // if the norm is greater than the specified minimum sampling distance or
  // if the change in rotation is greater than specified min.
  if (l2sqrd > dist * dist) {
    // then yes take a new scan
    return true;
  } else if (diffSq(0, 0) > minRotSq || diffSq(1, 0) > minRotSq ||
             diffSq(2, 0) > minRotSq) {
    // then yes take a new scan
    return true;
  } else {
    // then no, do not take a new scan
    return false;
  }
}

// Scan Matcher (parent Class) Functions:

void ScanMatcher::createPoseScanMap(boost::shared_ptr<ROSBag> ros_data) {
  LOG_INFO("Storing pose scans...");
  // save initial poses of the lidar scans based on GPS data and save iterators
  // corresponding to these scans
  int i = 0;
  Eigen::Affine3d T_ECEF_GPS, T_MAP_LIDAR;
  for (uint64_t iter = 0; iter < ros_data->lidar_container.size();
       iter++) { // this ierates through the lidar measurements
    bool use_next_scan = false;
    switch (this->params.init_method) {
    case 1:
      try {
        // extract gps measurement at the same timepoint as the current lidar
        // message
        auto gps_pose = ros_data->gps_container.get(
            ros_data->lidar_container[iter].time_point, 0);
        T_ECEF_GPS = gpsToEigen(gps_pose.first, true); // true: apply T_ENU_GPS
        T_MAP_LIDAR = ros_data->T_ECEF_MAP.inverse() * T_ECEF_GPS *
                      this->params.T_LIDAR_GPS.inverse();
      } catch (const std::out_of_range &e) {
        LOG_INFO("No gps pose for time of scan, may happen at edges of "
                 "recorded data");
        use_next_scan = true;
        break;
      }
      break;
    case 2:
      try {
        // extract odometry pose at the same timepoint as current lidar message
        auto odom_pose = ros_data->odom_container.get(
            ros_data->lidar_container[iter].time_point, 2);
        T_MAP_LIDAR = odom_pose.first;
      } catch (const std::out_of_range &e) {
        LOG_INFO("No odometry message for time of scan, may happen at edges of "
                 "recorded data");
        use_next_scan = true;
        break;
      }
      break;
    }

    // If i > 0 then check to see if the distance between current scan and
    // last scan is greater than the minimum, if so then save this pose
    // If i = 0, then save the scan - first scan
    if (i > 0) {
      bool take_new_scan;
      take_new_scan = takeNewScan(T_MAP_LIDAR, init_pose.poses[i - 1],
                                  this->params.trajectory_sampling_dist,
                                  this->params.trajectory_rotation_change);
      if (take_new_scan && !use_next_scan) {
        this->init_pose.poses.push_back(T_MAP_LIDAR);
        this->pose_scan_map.push_back(iter);
        ++i;
      }
    } else if (!use_next_scan) {
      this->init_pose.poses.push_back(T_MAP_LIDAR);
      this->pose_scan_map.push_back(iter);
      ++i;
    }
  }
  LOG_INFO("Stored %d pose scans of %d available scans.", i,
           ros_data->lidar_container.size());
}

void ScanMatcher::findAdjacentScans() {
  double distancejk;
  Eigen::Vector3d posej, posek;
  this->total_matches = 0; // reset the counter of matches

  // Scan registration class
  // As factors are added, factor id will just be counter value
  this->adjacency = boost::make_shared<std::vector<std::vector<uint64_t>>>(
      this->init_pose.poses.size());

  /*
   * For each pose within the initial pose, get the position.
   * Connect all the positions until the last pose by adding it to the adjacency
   * at that point
   *
   */
  for (uint64_t j = 0; j < this->init_pose.poses.size(); j++) {
    // posej is the position of the current pose
    posej(0, 0) = this->init_pose.poses[j](0, 3);
    posej(1, 0) = this->init_pose.poses[j](1, 3);
    posej(2, 0) = this->init_pose.poses[j](2, 3);

    // Do this for all poses except the last one
    if (j + 1 < this->init_pose.poses.size()) {
      // ensures that trajectory is connected
      //  this is also accounted for in nn
      //  search (see if j < ret_indices[k]-1)
      this->adjacency->at(j).emplace_back(j + 1);
      this->total_matches++;
    }

    /*
     * For each pose position, check the next scans to see if they are
     * within the boundaries specified
     * If they are and they are after j, add them to the adjacency
     *
     */
    for (uint16_t k = j + 2;
         (k < j + 1 + this->params.knn) && (k < this->init_pose.poses.size());
         k++) {
      posek(0, 0) = this->init_pose.poses[k](0, 3);
      posek(1, 0) = this->init_pose.poses[k](1, 3);
      posek(2, 0) = this->init_pose.poses[k](2, 3);
      distancejk = calculateLength(posej, posek);

      if ((distancejk > this->params.distance_match_min) &&
          (distancejk < this->params.distance_match_limit)) {
        // add index to back of vector for scan j
        this->adjacency->at(j).emplace_back(k);
        this->total_matches++;
      }
    }
  }
}

void ScanMatcher::findLoops() {
  int knn_ = this->params.knn;
  double pathLength, distanceP1P2;
  Eigen::Vector3d pose1, pose2, poseLast;
  std::vector<uint64_t> loopIndices = {0, 0};

  // creating a vector (size nx1) of vectors (will be size 2x1)
  this->loops = boost::make_shared<std::vector<std::vector<uint64_t>>>();

  for (uint64_t j = 0; j < this->init_pose.poses.size() - knn_; j++) {
    pathLength = 0;
    distanceP1P2 = 0;
    pose1(0, 0) = this->init_pose.poses[j](0, 3);
    pose1(1, 0) = this->init_pose.poses[j](1, 3);
    pose1(2, 0) = this->init_pose.poses[j](2, 3);
    poseLast = pose1;

    // For each pose (pose1) position, check all the poses j + 1 and up
    // if within loop_max_distance and outside of loop_min_travel_distance,
    // then add the constraint (or add to loops object)
    for (uint16_t k = j + 1; k < init_pose.poses.size(); k++) {
      pose2(0, 0) = this->init_pose.poses[k](0, 3);
      pose2(1, 0) = this->init_pose.poses[k](1, 3);
      pose2(2, 0) = this->init_pose.poses[k](2, 3);

      distanceP1P2 = calculateLength(pose1, pose2);
      pathLength += calculateLength(poseLast, pose2);
      poseLast = pose2;

      if (distanceP1P2 < this->params.loop_max_distance &&
          pathLength > this->params.loop_min_travel_distance) {
        loopIndices = {j, k};
        this->loops->emplace_back(loopIndices);
        this->total_matches++;
      }
    }
  }
  LOG_INFO("Found a total of %d loop closure scans.", this->loops->size());
}

void ScanMatcher::displayPointCloud(wave::PCLPointCloudPtr cloud_display,
                                    int color,
                                    const Eigen::Affine3d &transform) {
  if (this->params.visualize) {
    *this->cloud_temp_display = *cloud_display;
    if (!transform.matrix().isIdentity()) {
      pcl::transformPointCloud(*cloud_display, *this->cloud_temp_display,
                               transform);
    }
    this->init_display.addPointcloud(this->cloud_temp_display, color);
  }
}

void ScanMatcher::createAggregateMap(GTSAMGraph &graph,
                                     boost::shared_ptr<ROSBag> ros_data,
                                     int mapping_method) {
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> intermediaries;
  this->aggregate->clear();
  int i = 0;
  for (uint64_t k = 0; k < graph.poses.size();
       k++) // NOTE: Sometimes I get seg fault on the last scan
  {         // iterate through all poses in graph

    // Define transforms we will need
    Eigen::Affine3d T_MAP_LLOC_k, T_MAP_LLOC_kp1, T_MAP_LMAP_k, T_MAP_LMAP_kp1,
        T_LMAP_LLOC, T_MAP_LLOC_Jprev, T_MAP_LMAP_Jprev;
    T_MAP_LLOC_k = this->final_poses.at(graph.poses.at(k)).value; // for pose k
    T_LMAP_LLOC = this->params.T_LMAP_LLOC;                       // static
    T_MAP_LMAP_k = T_MAP_LLOC_k * T_LMAP_LLOC.inverse();

    int curr_index = this->pose_scan_map.at(graph.poses.at(k));
    TimePoint curr_pose_time = ros_data->lidar_container[curr_index].time_point;
    int next_index;
    TimePoint next_pose_time = curr_pose_time;

    // get all time and transforms for next pose for interpolation
    if (!(k == graph.poses.size() - 1)) {
      T_MAP_LLOC_kp1 =
          this->final_poses.at(graph.poses.at(k + 1)).value; // for pose k + 1
      T_MAP_LMAP_kp1 = T_MAP_LLOC_kp1 * T_LMAP_LLOC.inverse();
      T_MAP_LLOC_Jprev = T_MAP_LLOC_k;
      T_MAP_LMAP_Jprev = T_MAP_LMAP_k;
      next_index = this->pose_scan_map.at(graph.poses.at(k + 1));
      next_pose_time = ros_data->lidar_container[next_index].time_point;
    }

    // find scan range for map scans:
    std::pair<int, int> scan_range_map;
    scan_range_map = getLidarTimeWindow(ros_data->lidar_container_map,
                                        curr_pose_time, next_pose_time);

    switch (mapping_method) {
    case 1:
      // transform current pose scan to target cloud
      pcl::transformPointCloud(*(ros_data->lidar_container[curr_index].value),
                               *this->cloud_target, T_MAP_LLOC_k);

      // iterate through all scans between pose k and k+1
      if ((this->params.trajectory_sampling_dist >
           this->params.map_sampling_dist) &&
          !(k == graph.poses.size() - 1)) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_interp(
            new pcl::PointCloud<pcl::PointXYZ>);

        int j = 0;
        while (true) {
          j++;
          Eigen::Affine3d T_MAP_LLOC_J; // interpolated scan pose
          TimePoint time_point_J =
              ros_data->lidar_container[curr_index + j].time_point;

          if (time_point_J >= next_pose_time) // stop interpolation
          {
            break;
          }

          T_MAP_LLOC_J.matrix() = interpolateTransform(
              T_MAP_LLOC_k.matrix(), curr_pose_time, T_MAP_LLOC_kp1.matrix(),
              next_pose_time, time_point_J);

          bool take_new_map_scan = takeNewScan(
              T_MAP_LLOC_Jprev, T_MAP_LLOC_J, this->params.map_sampling_dist,
              this->params.map_rotation_change);

          // interpolate pose and add new scan to current target cloud
          if (take_new_map_scan) {
            pcl::transformPointCloud(
                *(ros_data->lidar_container[curr_index + j].value),
                *cloud_interp, T_MAP_LLOC_J);
            *cloud_target += *cloud_interp;
            T_MAP_LLOC_Jprev = T_MAP_LLOC_J;
          }
        }
      }
      break;

    case 2:
      // transform current pose scan to target cloud
      pcl::transformPointCloud(
          *(ros_data->lidar_container_map[scan_range_map.first].value),
          *this->cloud_target, T_MAP_LMAP_k);

      // iterate through all scans between pose k and k+1
      if ((this->params.trajectory_sampling_dist >
           this->params.map_sampling_dist) &&
          !(k == graph.poses.size() - 1)) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_interp(
            new pcl::PointCloud<pcl::PointXYZ>);

        int j = 0;
        while (true) {
          j++;
          Eigen::Affine3d T_MAP_LMAP_J; // interpolated scan pose
          TimePoint time_point_J =
              ros_data->lidar_container_map[scan_range_map.first + j]
                  .time_point;

          if (time_point_J >= next_pose_time) // stop interpolation
          {
            break;
          }

          T_MAP_LMAP_J.matrix() = interpolateTransform(
              T_MAP_LMAP_k.matrix(), curr_pose_time, T_MAP_LMAP_kp1.matrix(),
              next_pose_time, time_point_J);
          bool take_new_map_scan = takeNewScan(
              T_MAP_LMAP_Jprev, T_MAP_LMAP_J, this->params.map_sampling_dist,
              this->params.map_rotation_change);

          // interpolate pose and add new scan to current target cloud
          if (take_new_map_scan) {
            pcl::transformPointCloud(
                *(ros_data->lidar_container_map[scan_range_map.first + j]
                      .value),
                *cloud_interp, T_MAP_LMAP_J);
            *cloud_target += *cloud_interp;
            T_MAP_LMAP_Jprev = T_MAP_LMAP_J;
          }
        }
      }

      break;
    case 3:

      // transform current map scan to tmp cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(
          *(ros_data->lidar_container_map[scan_range_map.first].value),
          *cloud_tmp, T_MAP_LMAP_k);
      // transform current localization scan to target cloud
      pcl::transformPointCloud(*(ros_data->lidar_container[curr_index].value),
                               *this->cloud_target, T_MAP_LLOC_k);
      // Add tmp cloud to target cloud
      *this->cloud_target += *cloud_tmp;

      // iterate through all scans between pose k and k+1
      if ((this->params.trajectory_sampling_dist >
           this->params.map_sampling_dist) &&
          !(k == graph.poses.size() - 1)) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_interp(
            new pcl::PointCloud<pcl::PointXYZ>);
        int j = 0;
        while (true) {
          j++;
          Eigen::Affine3d T_MAP_LLOC_J, T_MAP_LMAP_J; // interpolated scan pose
          TimePoint time_point_J =
              ros_data->lidar_container[curr_index + j].time_point;

          if (time_point_J >= next_pose_time) // stop interpolation
          {
            break;
          }

          T_MAP_LLOC_J.matrix() = interpolateTransform(
              T_MAP_LLOC_k.matrix(), curr_pose_time, T_MAP_LLOC_kp1.matrix(),
              next_pose_time, time_point_J);
          T_MAP_LMAP_J.matrix() = interpolateTransform(
              T_MAP_LMAP_k.matrix(), curr_pose_time, T_MAP_LMAP_kp1.matrix(),
              next_pose_time, time_point_J);

          bool take_new_map_scan = takeNewScan(
              T_MAP_LLOC_Jprev, T_MAP_LLOC_J, this->params.map_sampling_dist,
              this->params.map_rotation_change);

          // interpolate pose and add new scan to current target cloud
          if (take_new_map_scan) {
            pcl::transformPointCloud(
                *(ros_data->lidar_container[curr_index + j].value),
                *cloud_interp, T_MAP_LLOC_J);
            *cloud_target += *cloud_interp;
            pcl::transformPointCloud(
                *(ros_data->lidar_container_map[scan_range_map.first + j]
                      .value),
                *cloud_interp, T_MAP_LMAP_J);
            *cloud_target += *cloud_interp;
            T_MAP_LLOC_Jprev = T_MAP_LLOC_J;
            T_MAP_LMAP_Jprev = T_MAP_LMAP_J;
          }
        }
      }

      break;
    }

    // this block makes sure every intermediate map is made up of n scans.
    // each scan is filtered individually when saved in measurements
    // containers, then the whole set of n
    // combined scans is filtered once (default, n=15)
    if ((k % this->params.int_map_size) ==
        0) { // every nth pose, filter the intermediate map if specified then
             // move to next
      if ((i != 0) &&
          !(this->params.downsample_output_method ==
            3)) { // if not first intermediate map, then filter it and
        // move to the next intermediate map
        *intermediaries.at(i) = downSampleFilterIG(
            intermediaries.at(i), this->params.downsample_cell_size);
        i++;
      } else // if first intermediate map
      {
        if (k != 0) { // if it's the first scan, do nothing.
                      // if it's not the first scan, but it is the first
                      // intermediate map , then increase iterator of int. maps
          i++;
        }
      }
      // add new empty point cloud to set of intermediate maps
      intermediaries.emplace_back(new pcl::PointCloud<pcl::PointXYZ>);
    }

    // filter each new cloud if specified in config
    if (this->params.downsample_output_method == 1) {
      *cloud_target = downSampleFilterIG(this->cloud_target,
                                         this->params.downsample_cell_size);
    }

    // add each new cloud to current intermediate map
    *(intermediaries.at(i)) += *this->cloud_target;
  }

  for (uint64_t iter = 0; iter < intermediaries.size(); iter++) {
    *this->aggregate += *(intermediaries.at(iter));
  }
}

void ScanMatcher::outputAggregateMap(GTSAMGraph &graph,
                                     boost::shared_ptr<ROSBag> ros_data,
                                     int mapping_method, std::string path_) {
  *this->aggregate =
      downSampleFilterIG(this->aggregate, this->params.downsample_cell_size);
  std::string dateandtime = convertTimeToDate(std::chrono::system_clock::now());
  std::string mapType;
  switch (mapping_method) {
  case 1:
    mapType = "_loc_map.pcd";
    break;
  case 2:
    mapType = "_map.pcd";
    break;
  case 3:
    mapType = "_comb_map.pcd";
    break;
  }

  pcl::io::savePCDFileBinary(path_ + dateandtime + mapType, *this->aggregate);
  LOG_INFO("outputting map at time: %s", dateandtime.c_str());

  // TODO: move this to another function
  // now save trajectory to file
  std::ofstream file;
  const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision,
                                         Eigen::DontAlignCols, ", ", ", ");
  file.open(path_ + dateandtime + "_opt_traj" + ".txt");
  for (auto iter = final_poses.begin(); iter != final_poses.end(); iter++) {
    file << iter->time_point.time_since_epoch().count() << ", ";
    file << iter->value.matrix().format(CSVFormat);
    file << std::endl;
  }
  file.close();

  std::ofstream bias_file;
  bias_file.open(path_ + dateandtime + "_GPSbias.txt");
  for (auto iter = graph.biases.begin(); iter != graph.biases.end(); iter++) {
    auto curbias = graph.result.at<gtsam::Point3>(*iter);
    bias_file << curbias.x() << ", " << curbias.y() << ", " << curbias.z()
              << std::endl;
  }
  bias_file.close();

  // TODO: Make this work even without GPS data
  if (this->params.init_method == 1) {
    // This file contains the input traj to the GTSAM since we want to compare
    // the pose difference betweem the input and the output of GTSAM
    std::ofstream gt_file;
    gt_file.open(path_ + dateandtime + "_GTSAMinputTraj.txt");
    for (uint64_t j = 0; j < adjacency->size(); j++) {
      Eigen::Affine3d T_ECEF_GPSIMU = ros_data->getGPSTransform(
          ros_data->getLidarScanTimePoint(pose_scan_map.at(j)), true);
      Eigen::Affine3d T_MAP_GPSIMU, T_MAP_LIDAR;
      T_MAP_GPSIMU = ros_data->T_ECEF_MAP.inverse() * T_ECEF_GPSIMU;
      if (this->params.optimize_gps_lidar) {
        auto result = graph.result.at<gtsam::Pose3>(6000000);
        T_MAP_LIDAR = T_MAP_GPSIMU * result.matrix();
      } else {
        auto result = params.T_LIDAR_GPS.inverse();
        T_MAP_LIDAR = T_MAP_GPSIMU * result.matrix();
      }
      gt_file << ros_data->getLidarScanTimePoint(pose_scan_map.at(j))
                     .time_since_epoch()
                     .count()
              << ", ";
      gt_file << T_MAP_LIDAR.matrix().format(CSVFormat);
      gt_file << std::endl;
    }
    gt_file.close();

    std::ofstream datumfile;
    using dbl = std::numeric_limits<double>;
    datumfile.open(path_ + dateandtime + "map_ecef_datum" + ".txt");
    datumfile.precision(dbl::max_digits10);
    datumfile << ros_data->T_ECEF_MAP.matrix().format(CSVFormat);
    datumfile.close();
    if (this->params.optimize_gps_lidar) {
      auto result = graph.result.at<gtsam::Pose3>(6000000);
      std::ofstream datumfile;
      using dbl = std::numeric_limits<double>;
      datumfile.open(dateandtime + "T_LIDAR_GPS" + ".txt");
      datumfile.precision(dbl::max_digits10);
      datumfile << result.inverse().matrix().format(CSVFormat);
      datumfile.close();
    }
  }
}

// ICPScanMatcher (Child Class) Functions
ICPScanMatcher::ICPScanMatcher(Params &p_, std::string matcherConfigPath)
    : ScanMatcher(p_),
      // segmenter(this->seg_params),
      matcher(wave::ICPMatcherParams(matcherConfigPath)) {
  this->params = p_;
  this->cloud_ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  this->cloud_tgt = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

bool ICPScanMatcher::matchScans(
    uint64_t i, uint64_t j, Eigen::Affine3d &T_Li_Lj, wave::Mat6 &info,
    bool &correction_norm_valid,
    boost::shared_ptr<ROSBag> ros_data) { // j: current, (reference scan)
  // i: adjacent scan (target)
  auto T_MAP_Lj = init_pose.poses[j]; // set initial guess of current scan
  auto T_MAP_Li = init_pose.poses[i]; // set initial guess of adjacent scan
  correction_norm_valid = true;

  // Get scans
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref2(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt2(
      new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ref2 = *ros_data->lidar_container[pose_scan_map.at(j)].value;
  *cloud_tgt2 = *ros_data->lidar_container[pose_scan_map.at(i)].value;
  TimePoint timepoint_j, timepoint_i;

  // Combine sacns if specified
  if (this->params.combine_scans) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref_tmp(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_tmp(
        new pcl::PointCloud<pcl::PointXYZ>);
    timepoint_j = ros_data->lidar_container[pose_scan_map.at(j)].time_point;
    timepoint_i = ros_data->lidar_container[pose_scan_map.at(i)].time_point;
    int map_index_j =
        getLidarTimeWindow(ros_data->lidar_container_map, timepoint_j);
    int map_index_i =
        getLidarTimeWindow(ros_data->lidar_container_map, timepoint_i);
    pcl::transformPointCloud(
        *(ros_data->lidar_container_map[map_index_j].value), *cloud_ref_tmp,
        this->params.T_LMAP_LLOC.inverse());
    pcl::transformPointCloud(
        *(ros_data->lidar_container_map[map_index_i].value), *cloud_tgt_tmp,
        this->params.T_LMAP_LLOC.inverse());
    *cloud_ref2 += *cloud_ref_tmp;
    *cloud_tgt2 += *cloud_tgt_tmp;
  }

  // set current scan as reference scan
  this->matcher.setRef(cloud_ref2);

  // display reference scan
  this->displayPointCloud(cloud_ref2, 0); // white

  // calculate init. T from adjacent to current
  auto T_estLj_Li = T_MAP_Lj.inverse() * T_MAP_Li;

  // transform adjacent scan (i) to estimated current scan frame using
  // initialized T, then assign to cloud_target
  pcl::transformPointCloud(*cloud_tgt2, *(this->cloud_tgt), T_estLj_Li);

  this->matcher.setTarget(this->cloud_tgt);
  this->displayPointCloud(this->cloud_tgt, 1); // red
  if (matcher.match()) {
    auto T_estLj_Lj =
        this->matcher
            .getResult(); // assign estimated transform to new current position
    this->displayPointCloud(cloud_tgt, 2, T_estLj_Lj.inverse()); // blue
    if (this->params.visualize && this->params.step_matches) {
      std::cin.get(); // wait for user to hit next
    }
    double delta = T_estLj_Lj.matrix().block(0, 3, 3, 1).norm();
    double total = T_estLj_Li.matrix().block(0, 3, 3, 1).norm();

    if (this->params.use_gps & ((delta / total) > 0.4)) {
      LOG_INFO("Correction norm %f %%", 100.0 * delta / total);
      correction_norm_valid = false;
      return false;
    }

    T_Li_Lj = T_estLj_Li.inverse() *
              T_estLj_Lj; // correct estimated T by match result

    // try scaling info by how much correction was required
    if (this->params.fixed_scan_transform_cov) {
      info = this->params.scan_transform_cov;
    } else {
      matcher.estimateInfo();
      info = this->matcher.getInfo(); //(0.1 / (0.1 + delta)) *
    }
    return true;
  } else {
    return false;
  }
}

// GICPScanMatcher (Child Class) Functions
GICPScanMatcher::GICPScanMatcher(Params &p_, std::string matcherConfigPath)
    : ScanMatcher(p_), matcher(wave::GICPMatcherParams(matcherConfigPath)) {
  this->params = p_;
  this->cloud_ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  this->cloud_tgt = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

bool GICPScanMatcher::matchScans(
    uint64_t i, uint64_t j, Eigen::Affine3d &T_Li_Lj, wave::Mat6 &info,
    bool &correction_norm_valid,
    boost::shared_ptr<ROSBag> ros_data) { // j: current, (reference scan)
  // i: adjacent scan (target)
  auto T_MAP_Lj = init_pose.poses[j]; // set initial guess of current scan
  auto T_MAP_Li = init_pose.poses[i]; // set initial guess of adjacent scan
  correction_norm_valid = true;

  // Get scans
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref2(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt2(
      new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ref2 = *ros_data->lidar_container[pose_scan_map.at(j)].value;
  *cloud_tgt2 = *ros_data->lidar_container[pose_scan_map.at(i)].value;
  TimePoint timepoint_j, timepoint_i;

  // Combine sacns if specified
  if (this->params.combine_scans) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref_tmp(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_tmp(
        new pcl::PointCloud<pcl::PointXYZ>);
    timepoint_j = ros_data->lidar_container[pose_scan_map.at(j)].time_point;
    timepoint_i = ros_data->lidar_container[pose_scan_map.at(i)].time_point;
    int map_index_j =
        getLidarTimeWindow(ros_data->lidar_container_map, timepoint_j);
    int map_index_i =
        getLidarTimeWindow(ros_data->lidar_container_map, timepoint_i);
    pcl::transformPointCloud(
        *(ros_data->lidar_container_map[map_index_j].value), *cloud_ref_tmp,
        this->params.T_LMAP_LLOC.inverse());
    pcl::transformPointCloud(
        *(ros_data->lidar_container_map[map_index_i].value), *cloud_tgt_tmp,
        this->params.T_LMAP_LLOC.inverse());
    *cloud_ref2 += *cloud_ref_tmp;
    *cloud_tgt2 += *cloud_tgt_tmp;
  }

  // set current scan as reference scan
  this->matcher.setRef(cloud_ref2);

  // display reference scan
  this->displayPointCloud(cloud_ref2, 0); // white

  // calculate init. T from adjacent to current
  auto T_estLj_Li = T_MAP_Lj.inverse() * T_MAP_Li;

  // transform adjacent scan (i) to estimated current scan frame using
  // initialized T, then assign to cloud_target
  pcl::transformPointCloud(*cloud_tgt2, *(this->cloud_tgt), T_estLj_Li);

  this->matcher.setTarget(this->cloud_tgt);
  this->displayPointCloud(this->cloud_tgt, 1); // red
  if (matcher.match()) {
    auto T_estLj_Lj =
        this->matcher
            .getResult(); // assign estimated transform to new current position
    this->displayPointCloud(cloud_tgt, 2, T_estLj_Lj.inverse()); // blue
    if (this->params.visualize && this->params.step_matches) {
      std::cin.get(); // wait for user to hit next
    }
    double delta = T_estLj_Lj.matrix().block(0, 3, 3, 1).norm();
    double total = T_estLj_Li.matrix().block(0, 3, 3, 1).norm();

    if (this->params.use_gps & ((delta / total) > 0.4)) {
      LOG_INFO("Correction norm %f %%", 100.0 * delta / total);
      correction_norm_valid = false;
      return false;
    }

    T_Li_Lj = T_estLj_Li.inverse() *
              T_estLj_Lj; // correct estimated T by match result

    // try scaling info by how much correction was required
    if (this->params.fixed_scan_transform_cov) {
      info = this->params.scan_transform_cov;
    } else {
      matcher.estimateInfo();
      info = this->matcher.getInfo(); //(0.1 / (0.1 + delta)) *
    }
    return true;
  } else {
    return false;
  }
}
