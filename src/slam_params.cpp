#include "slam_params.hpp"

Params::Params(std::string &config_file_path) {
  wave::ConfigParser parser;
  parser.addParam("gps_type", &(this->gps_type));
  parser.addParam("gps_topic", &(this->gps_topic));
  parser.addParam("gps_frame", &(this->gps_frame));
  parser.addParam("imu_topic", &(this->imu_topic));
  parser.addParam("odom_topic", &(this->odom_topic));
  parser.addParam("odom_frame", &(this->odom_frame));
  parser.addParam("init_method", &(this->init_method));
  parser.addParam("init_files_path", &(this->init_files_path));
  parser.addParam("lidar_topic_loc", &(this->lidar_topic_loc));
  parser.addParam("lidar_topic_map", &(this->lidar_topic_map));
  parser.addParam("lidar_frame_loc", &(this->lidar_frame_loc));
  parser.addParam("lidar_frame_map", &(this->lidar_frame_map));
  parser.addParam("bag_file_path", &(this->bag_file_path));
  parser.addParam("use_prev_poses", &(this->use_prev_poses));
  parser.addParam("prev_poses_path", &(this->prev_poses_path));
  parser.addParam("extrinsics_filename", &(this->extrinsics_filename));
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

  // get config file path
  std::string yamlDirStr;
  if (config_file_path == "") {
    yamlDirStr = __FILE__;
    yamlDirStr.erase(yamlDirStr.end() - 19, yamlDirStr.end());
    yamlDirStr += "config/ig_graph_slam_config.yaml";
    this->config_file_path = yamlDirStr;
  } else {
    yamlDirStr = config_file_path;
    this->config_file_path = yamlDirStr;
  }
  LOG_INFO("Loading config file: %s", yamlDirStr.c_str());
  std::ifstream fileName(yamlDirStr.c_str());

  if (fileName.good()) {
    parser.load(yamlDirStr);
  } else {
    LOG_ERROR("Config file not found.");
  }

  LOG_INFO("Input Bag File: %s", this->bag_file_path.c_str());

  this->topics.push_back(this->lidar_topic_loc);
  this->topics.push_back(this->lidar_topic_map);
  this->topics.push_back(this->gps_topic);
  this->topics.push_back(this->imu_topic);
  this->topics.push_back(this->odom_topic);
}

void Params::outputParams() {
  std::cout << "----------------------------"
            << "\n"
            << "Outputting all parameters:"
            << "\n"
            << "----------------------------"
            << "\n"
            << "gps_type: " << this->gps_type << "\n"
            << "gps_topic: " << this->gps_topic << "\n"
            << "imu_topic: " << this->imu_topic << "\n"
            << "odom_topic: " << this->odom_topic << "\n"
            << "init_method: " << this->init_method << "\n"
            << "init_files_path: " << this->init_files_path << "\n"
            << "lidar_topic_loc: " << this->lidar_topic_loc << "\n"
            << "lidar_topic_map: " << this->lidar_topic_map << "\n"
            << "bag_file_path: " << this->bag_file_path << "\n"
            << "use_prev_poses: " << this->use_prev_poses << "\n"
            << "extrinsics_filename: " << this->extrinsics_filename << "\n"
            << "prev_poses_path: " << this->prev_poses_path << "\n"
            << "use_pass_through_filter: " << this->use_pass_through_filter
            << "\n"
            << "x_upper_threshold: " << this->x_upper_threshold << "\n"
            << "x_lower_threshold: " << this->x_lower_threshold << "\n"
            << "y_upper_threshold: " << this->y_upper_threshold << "\n"
            << "y_lower_threshold: " << this->y_lower_threshold << "\n"
            << "z_upper_threshold: " << this->z_upper_threshold << "\n"
            << "z_lower_threshold: " << this->z_lower_threshold << "\n"
            << "downsample_input: " << this->downsample_input << "\n"
            << "input_downsample_size: " << this->input_downsample_size << "\n"
            << "use_rad_filter: " << this->use_rad_filter << "\n"
            << "set_min_neighbours: " << this->set_min_neighbours << "\n"
            << "set_search_radius: " << this->set_search_radius << "\n"
            << "ground_segment: " << this->ground_segment << "\n"
            << "downsample_output_method: " << this->downsample_output_method
            << "\n"
            << "downsample_cell_size: " << this->downsample_cell_size << "\n"
            << "int_map_size: " << this->int_map_size << "\n"
            << "use_pass_through_filter_map: "
            << this->use_pass_through_filter_map << "\n"
            << "x_upper_threshold_map: " << this->x_upper_threshold_map << "\n"
            << "x_lower_threshold_map: " << this->x_lower_threshold_map << "\n"
            << "y_upper_threshold_map: " << this->y_upper_threshold_map << "\n"
            << "y_lower_threshold_map: " << this->y_lower_threshold_map << "\n"
            << "z_upper_threshold_map: " << this->z_upper_threshold_map << "\n"
            << "z_lower_threshold_map: " << this->z_lower_threshold_map << "\n"
            << "k_nearest_neighbours: " << this->knn << "\n"
            << "trajectory_sampling_distance: "
            << this->trajectory_sampling_dist << "\n"
            << "map_sampling_distance: " << this->map_sampling_dist << "\n"
            << "distance_match_min: " << this->distance_match_min << "\n"
            << "distance_match_limit: " << this->distance_match_limit << "\n"
            << "rotation_match_min: " << this->rotation_match_min << "\n"
            << "rotation_match_limit: " << this->rotation_match_limit << "\n"
            << "loop_max_distance: " << this->loop_max_distance << "\n"
            << "loop_min_travel_distance: " << this->loop_min_travel_distance
            << "\n"
            << "iterations: " << this->iterations << "\n"
            << "use_gps: " << this->use_gps << "\n"
            << "optimize_gps_lidar: " << this->optimize_gps_lidar << "\n"
            << "fixed_scan_transform_cov: " << this->fixed_scan_transform_cov
            << "\n"
            << "visualize: " << this->visualize << "\n"
            << "step_matches: " << this->step_matches << "\n"
            << "combine_scans: " << this->combine_scans << "\n"
            << "matcher_type: " << this->matcher_type << "\n"
            << "output_path: " << this->output_path << "\n"
            << "----------------------------"
            << "\n";
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

  if (this->gps_frame == "") {
    LOG_ERROR("Please enter GPS frame.");
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

  if (!(this->init_method == 1 || this->init_method == 2 ||
        this->init_method == 3)) {
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

  if (this->lidar_frame_loc == "") {
    LOG_ERROR("Please enter localization lidar frame.");
    return 0;
  }

  if (this->lidar_topic_map == "") {
    LOG_ERROR("Please enter lidar map frame.");
    return 0;
  }

  if (!boost::filesystem::exists(this->bag_file_path) && init_method != 3) {
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
        this->downsample_output_method == 3 ||
        this->downsample_output_method == 4)) {
    LOG_ERROR(
        "Invalid parameter: downsample_output_method. Enter 1, 2, 3 or 4.");
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
