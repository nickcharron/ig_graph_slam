#include "slam_params.hpp"
#include "utils.hpp"
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <wave/utils/config.hpp>
#include <wave/utils/log.hpp>
#include <wave/utils/math.hpp>

Params::Params() {
  wave::ConfigParser parser;
  parser.addParam("gps_type", &(this->gps_type));
  parser.addParam("gps_topic", &(this->gps_topic));
  parser.addParam("imu_topic", &(this->imu_topic));
  parser.addParam("odom_topic", &(this->odom_topic));
  parser.addParam("init_method", &(this->init_method));
  parser.addParam("lidar_topic_loc", &(this->lidar_topic_loc));
  parser.addParam("lidar_topic_map", &(this->lidar_topic_map));
  parser.addParam("bag_file_path", &(this->bag_file_path));
  parser.addParam("use_prev_poses", &(this->use_prev_poses));
  parser.addParam("prev_poses_path", &(this->prev_poses_path));
  parser.addParam("camera_topics", &(params.camera_topics));
  parser.addParam("intrinsics", &(params.intrinsics));
  parser.addParam("T_LIDAR_GPS", &(this->T_LIDAR_GPS.matrix()));
  parser.addParam("T_LMAP_LLOC", &(this->T_LMAP_LLOC.matrix()));
  parser.addParam("use_pass_through_filter", &(this->use_pass_through_filter));
  parser.addParam("x_upper_threshold", &(this->x_upper_threshold));
  parser.addParam("x_lower_threshold", &(this->x_lower_threshold));
  parser.addParam("y_upper_threshold", &(this->y_upper_threshold));
  parser.addParam("y_lower_threshold", &(this->y_lower_threshold));
  parser.addParam("z_upper_threshold", &(this->z_upper_threshold));
  parser.addParam("z_lower_threshold", &(this->z_lower_threshold));
  parser.addParam("downsample_input", &(this->downsample_input));
  parser.addParam("input_downsample_size", &(this->input_downsample_size));
  parser.addParam("use_rad_filter", &(this->use_rad_filter));
  parser.addParam("set_min_neighbours", &(this->set_min_neighbours));
  parser.addParam("set_search_radius", &(this->set_search_radius));
  parser.addParam("ground_segment", &(this->ground_segment));
  parser.addParam("downsample_output_method",
                  &(this->downsample_output_method));
  parser.addParam("downsample_cell_size", &(this->downsample_cell_size));
  parser.addParam("int_map_size", &(this->int_map_size));
  parser.addParam("use_pass_through_filter_map",
                  &(this->use_pass_through_filter_map));
  parser.addParam("x_upper_threshold_map", &(this->x_upper_threshold_map));
  parser.addParam("x_lower_threshold_map", &(this->x_lower_threshold_map));
  parser.addParam("y_upper_threshold_map", &(this->y_upper_threshold_map));
  parser.addParam("y_lower_threshold_map", &(this->y_lower_threshold_map));
  parser.addParam("z_upper_threshold_map", &(this->z_upper_threshold));
  parser.addParam("z_lower_threshold_map", &(this->z_lower_threshold_map));
  parser.addParam("k_nearest_neighbours", &(this->knn));
  parser.addParam("trajectory_sampling_distance",
                  &(this->trajectory_sampling_dist));
  parser.addParam("map_sampling_distance", &(this->map_sampling_dist));
  parser.addParam("trajectory_rotation_change",
                  &(this->trajectory_rotation_change));
  parser.addParam("map_rotation_change", &(this->map_rotation_change));
  parser.addParam("distance_match_min", &(this->distance_match_min));
  parser.addParam("distance_match_limit", &(this->distance_match_limit));
  parser.addParam("rotation_match_min", &(this->rotation_match_min));
  parser.addParam("rotation_match_limit", &(this->rotation_match_limit));
  parser.addParam("loop_max_distance", &(this->loop_max_distance));
  parser.addParam("loop_min_travel_distance",
                  &(this->loop_min_travel_distance));
  parser.addParam("iterations", &(this->iterations));
  parser.addParam("use_gps", &(this->use_gps));
  parser.addParam("optimize_gps_lidar", &(this->optimize_gps_lidar));
  parser.addParam("fixed_scan_transform_cov",
                  &(this->fixed_scan_transform_cov));
  parser.addParam("scan_transform_cov", &(this->scan_transform_cov));
  parser.addParam("visualize", &(this->visualize));
  parser.addParam("step_matches", &(this->step_matches));
  parser.addParam("combine_scans", &(this->combine_scans));
  parser.addParam("matcher_type", &(this->matcher_type));
  parser.addParam("output_path", &(this->output_path));

  std::string yamlDirStr = __FILE__;
  yamlDirStr.erase(yamlDirStr.end() - 19, yamlDirStr.end());
  yamlDirStr += "config/ig_graph_slam_config.yaml";
  this->config_file_path = yamlDirStr;
  std::ifstream fileName(yamlDirStr.c_str());

  if (fileName.good()) {
    parser.load(yamlDirStr);
  } else {
    LOG_ERROR("ig_graph_slam.yaml not found in config folder");
  }

  this->topics.push_back(this->lidar_topic_loc);
  this->topics.push_back(this->lidar_topic_map);
  this->topics.push_back(this->gps_topic);
  this->topics.push_back(this->imu_topic);
  this->topics.push_back(this->odom_topic);
}

void Params::outputParams() {
  std::cout << "----------------------------" << std::endl
            << "Outputting all parameters:" << std::endl
            << "----------------------------" << std::endl
            << "gps_type: " << this->gps_type << std::endl
            << "gps_topic: " << this->gps_topic << std::endl
            << "imu_topic: " << this->imu_topic << std::endl
            << "odom_topic: " << this->odom_topic << std::endl
            << "init_method: " << this->init_method << std::endl
            << "lidar_topic_loc: " << this->lidar_topic_loc << std::endl
            << "lidar_topic_map: " << this->lidar_topic_map << std::endl
            << "bag_file_path: " << this->bag_file_path << std::endl
            << "use_prev_poses: " << this->use_prev_poses << std::endl
            << "prev_poses_path: " << this->prev_poses_path << std::endl;
  for (uint16_t i = 0; i < p_->camera_topics.size(); i++) {
    std::cout << "camera_topics: " << p_->camera_topics[i] << std::endl;
  }
  for (uint16_t i = 0; i < p_->camera_topics.size(); i++) {
    std::cout << "intrinsics: " << p_->intrinsics[i] << std::endl;
  }
  std::cout
      << "T_LIDAR_GPS: " << this->T_LIDAR_GPS.matrix() << std::endl
      << "T_LMAP_LLOC: " << this->T_LMAP_LLOC.matrix() << std::endl
      << "use_pass_through_filter: " << this->use_pass_through_filter
      << std::endl
      << "x_upper_threshold: " << this->x_upper_threshold << std::endl
      << "x_lower_threshold: " << this->x_lower_threshold << std::endl
      << "y_upper_threshold: " << this->y_upper_threshold << std::endl
      << "y_lower_threshold: " << this->y_lower_threshold << std::endl
      << "z_upper_threshold: " << this->z_upper_threshold << std::endl
      << "z_lower_threshold: " << this->z_lower_threshold << std::endl
      << "downsample_input: " << this->downsample_input << std::endl
      << "input_downsample_size: " << this->input_downsample_size << std::endl
      << "use_rad_filter: " << this->use_rad_filter << std::endl
      << "set_min_neighbours: " << this->set_min_neighbours << std::endl
      << "set_search_radius: " << this->set_search_radius << std::endl
      << "ground_segment: " << this->ground_segment << std::endl
      << "downsample_output_method: " << this->downsample_output_method
      << std::endl
      << "downsample_cell_size: " << this->downsample_cell_size << std::endl
      << "int_map_size: " << this->int_map_size << std::endl
      << "use_pass_through_filter_map: " << this->use_pass_through_filter_map
      << std::endl
      << "x_upper_threshold_map: " << this->x_upper_threshold_map << std::endl
      << "x_lower_threshold_map: " << this->x_lower_threshold_map << std::endl
      << "y_upper_threshold_map: " << this->y_upper_threshold_map << std::endl
      << "y_lower_threshold_map: " << this->y_lower_threshold_map << std::endl
      << "z_upper_threshold_map: " << this->z_upper_threshold_map << std::endl
      << "z_lower_threshold_map: " << this->z_lower_threshold_map << std::endl
      << "k_nearest_neighbours: " << this->knn << std::endl
      << "trajectory_sampling_distance: " << this->trajectory_sampling_dist
      << std::endl
      << "map_sampling_distance: " << this->map_sampling_dist << std::endl
      << "distance_match_min: " << this->distance_match_min << std::endl
      << "distance_match_limit: " << this->distance_match_limit << std::endl
      << "rotation_match_min: " << this->rotation_match_min << std::endl
      << "rotation_match_limit: " << this->rotation_match_limit << std::endl
      << "loop_max_distance: " << this->loop_max_distance << std::endl
      << "loop_min_travel_distance: " << this->loop_min_travel_distance
      << std::endl
      << "iterations: " << this->iterations << std::endl
      << "use_gps: " << this->use_gps << std::endl
      << "optimize_gps_lidar: " << this->optimize_gps_lidar << std::endl
      << "fixed_scan_transform_cov: " << this->fixed_scan_transform_cov
      << std::endl
      << "visualize: " << this->visualize << std::endl
      << "step_matches: " << this->step_matches << std::endl
      << "combine_scans: " << this->combine_scans << std::endl
      << "matcher_type: " << this->matcher_type << std::endl
      << "output_path: " << this->output_path << std::endl
      << "----------------------------" << std::endl;
}

std::string Params::getMatcherConfig() {
  std::string matcherConfigPath = __FILE__;
  matcherConfigPath.erase(matcherConfigPath.end() - 19,
                          matcherConfigPath.end());
  if (this->matcher_type == "icp") {
    matcherConfigPath += "config/icp.yaml";
    return matcherConfigPath;
  } else if (this->matcher_type == "loam") {
    LOG_ERROR("%s matcher type is not yet implemented. Coming soon.",
              this->matcher_type.c_str());
    return "";
  } else if (this->matcher_type == "gicp") {
    matcherConfigPath += "config/gicp.yaml";
    return matcherConfigPath;
  } else {
    LOG_ERROR("%s is not a valid matcher type. Change matcher type in "
              "ig_graph_slam_config.yaml",
              this->matcher_type.c_str());
    return "";
  }
}

bool Params::validateParams() {
  // NOTE: These checks need to be performed in the same order as the
  // fillparams function, or else you may not catch the first error.
  if (!(this->gps_type == "NavSatFix" || this->gps_type == "INSPVAX")) {
    if (this->init_method == 1) {
      LOG_ERROR("Invalid GPS type. Supported: NavSatFix, INSPVAX");
      return 0;
    }
  }

  if (this->gps_topic == "" && this->init_method == 1) {
    LOG_ERROR("Please enter GPS topic.");
    return 0;
  }

  if (this->imu_topic == "" && this->init_method == 1 &&
      this->gps_type == "NavSatFix") {
    LOG_ERROR("Please enter GPS/IMU topic.");
    return 0;
  }

  if (this->odom_topic == "" && this->init_method == 2) {
    LOG_ERROR("Please enter odometry topic.");
    return 0;
  }

  if (!(this->init_method == 1 || this->init_method == 2)) {
    LOG_ERROR("Invalid initialization method. Enter 1 or 2.");
    return 0;
  }

  if (this->lidar_topic_loc == "") {
    LOG_ERROR("Please enter localization lidar topic.");
    return 0;
  }

  if (this->lidar_topic_map == "") {
    LOG_INFO("WARNING: No lidar map topic.");
  }

  if (!boost::filesystem::exists(this->bag_file_path)) {
    LOG_ERROR("Cannot find bag file.");
    return 0;
  }

  if (!(this->use_prev_poses == 1 || this->use_prev_poses == 0)) {
    LOG_ERROR("Invalid parameter: use_prev_poses. Enter a boolean.");
    return 0;
  }

  if (!boost::filesystem::exists(this->prev_poses_path) &&
      this->use_prev_poses == 1) {
    LOG_ERROR("Cannot find previous poses file.");
    return 0;
  }

  if (p_->camera_topics.size() != p_->intrinsics.size()) {
    LOG_ERROR(
        "Number of camera topics not equal to number of intrinsic files.");
    return 0;
  }

  if (!isTransformationMatrix(this->T_LIDAR_GPS.matrix())) {
    LOG_ERROR("Invalid transformation matrix: T_LIDAR_GPS. Did not pass "
              "transformation check.");
    return 0;
  }

  if (!isTransformationMatrix(this->T_LMAP_LLOC.matrix())) {
    LOG_ERROR("Invalid transformation matrix: T_LMAP_LLOC. Did not pass "
              "transformation check.");
    return 0;
  }

  if (!(this->use_pass_through_filter == 1 ||
        this->use_pass_through_filter == 0)) {
    LOG_ERROR("Invalid parameter: use_pass_through_filter. Enter a boolean.");
    return 0;
  }

  if (!(this->downsample_input == 1 || this->downsample_input == 0)) {
    LOG_ERROR("Invalid parameter: downsample_input. Enter a boolean.");
    return 0;
  }

  if (!(this->use_rad_filter == 1 || this->use_rad_filter == 0)) {
    LOG_ERROR("Invalid parameter: downsample_input. Enter a boolean.");
    return 0;
  }

  if (this->set_min_neighbours == 0) {
    LOG_ERROR("Invalid parameter: set_min_neighbours. Enter an integer greater "
              "than 0.");
    return 0;
  }

  if (!(this->ground_segment == 1 || this->ground_segment == 0)) {
    LOG_ERROR("Invalid parameter: ground_segment. Enter a boolean.");
    return 0;
  }

  if (!(this->downsample_output_method == 1 ||
        this->downsample_output_method == 2 ||
        this->downsample_output_method == 3)) {
    LOG_ERROR("Invalid parameter: downsample_output_method. Enter 1, 2, or 3.");
    return 0;
  }

  if (this->int_map_size == 0) {
    LOG_ERROR(
        "Invalid parameter: int_map_size. Enter an integer greater than 0.");
    return 0;
  }

  if (!(this->use_pass_through_filter_map == 1 ||
        this->use_pass_through_filter_map == 0)) {
    LOG_ERROR(
        "Invalid parameter: use_pass_through_filter_map. Enter a boolean.");
    return 0;
  }

  if (this->x_upper_threshold < this->x_lower_threshold ||
      this->y_upper_threshold < this->y_lower_threshold ||
      this->z_upper_threshold < this->z_lower_threshold ||
      this->x_upper_threshold_map < this->x_lower_threshold_map ||
      this->y_upper_threshold_map < this->y_lower_threshold_map ||
      this->z_upper_threshold_map < this->z_lower_threshold_map) {
    LOG_ERROR("Check pass through filter parameters. Limits not correct.");
    return 0;
  }

  if (this->knn == 0) {
    LOG_ERROR("Invalid parameter: k_nearest_neighbours. Enter an integer "
              "greater than 0.");
    return 0;
  }

  if (this->loop_min_travel_distance < this->distance_match_limit) {
    LOG_ERROR("Parameter loop_min_travel_distance must be greater than "
              "parameter distance_match_limit");
    return 0;
  }

  if (this->trajectory_sampling_dist < this->map_sampling_dist) {
    LOG_ERROR("parameter trajectory_sampling_distance must be greater or equal "
              "to parameter map_sampling_distance");
    return 0;
  }

  if (this->distance_match_limit < this->distance_match_min) {
    LOG_ERROR("parameter distance_match_limit must be greater than parameter "
              "distance_match_min");
    return 0;
  }

  if (this->trajectory_rotation_change < 5) {
    LOG_ERROR(
        "parameter trajectory_rotation_change invalid. Might be too small");
    return 0;
  }

  if (this->map_rotation_change < 3) {
    LOG_ERROR("parameter map_rotation_change invalid. Might be too small");
    return 0;
  }

  if (this->rotation_match_min > this->rotation_match_limit) {
    LOG_ERROR("Parameter rotation_match_limit must be greater than "
              "parameter rotation_match_min");
    return 0;
  }

  if (this->rotation_match_min < 5) {
    LOG_ERROR("Parameter rotation_match_min is too small. Min = 5deg.");
    return 0;
  }

  if (this->rotation_match_limit > 360) {
    LOG_ERROR("Parameter rotation_match_limit is too large. Max = 360deg.");
    return 0;
  }

  if (this->iterations == 0) {
    LOG_ERROR(
        "Invalid parameter: iterations. Enter an integer greater than 0.");
    return 0;
  }

  if (!(this->use_gps == 1 || this->use_gps == 0)) {
    LOG_ERROR("Invalid parameter: use_gps. Enter a boolean.");
    return 0;
  }

  if (!(this->optimize_gps_lidar == 1 || this->optimize_gps_lidar == 0)) {
    LOG_ERROR("Invalid parameter: optimize_gps_lidar. Enter a boolean.");
    return 0;
  }

  if (!(this->fixed_scan_transform_cov == 1 ||
        this->fixed_scan_transform_cov == 0)) {
    LOG_ERROR("Invalid parameter: fixed_scan_transform_cov. Enter a boolean.");
    return 0;
  }

  if (!(this->visualize == 1 || this->visualize == 0)) {
    LOG_ERROR("Invalid parameter: visualize. Enter a boolean.");
    return 0;
  }

  if (!(this->step_matches == 1 || this->step_matches == 0)) {
    LOG_ERROR("Invalid parameter: step_matches. Enter a boolean.");
    return 0;
  }

  if (!(this->combine_scans == 1 || this->combine_scans == 0)) {
    LOG_ERROR("Invalid parameter: combine_scans. Enter a boolean.");
    return 0;
  }

  if (!(this->matcher_type == "icp" || this->matcher_type == "gicp" ||
        this->matcher_type == "loam")) {
    if (this->matcher_type == "loam") {
      LOG_ERROR("LOAM Matcher not yet implemented");
    } else {
      LOG_ERROR("Invalid parameter: matcher_type. Supported: icp, gicp, loam");
    }
    return 0;
  }

  std::string matcherConfigFilePath = getMatcherConfig();

  if (!boost::filesystem::exists(matcherConfigFilePath)) {
    if (this->matcher_type == "icp") {
      LOG_ERROR("icp.yaml not found. Looking in: %s",
                matcherConfigFilePath.c_str());
    } else if (this->matcher_type == "gicp") {
      LOG_ERROR("gicp.yaml not found. Looking in: %s",
                matcherConfigFilePath.c_str());
    } else if (this->matcher_type == "loam") {
      LOG_ERROR("loam.yaml not found. Looking in: %s",
                matcherConfigFilePath.c_str());
    } else {
      LOG_ERROR("matcher config file not found. Looking in: %s",
                matcherConfigFilePath.c_str());
    }
    return 0;
  }
  return 1;
}
