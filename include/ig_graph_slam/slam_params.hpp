#ifndef IG_GRAPH_SLAM_SLAM_PARAMS_HPP
#define IG_GRAPH_SLAM_SLAM_PARAMS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <wave/utils/math.hpp>

/***
 * Params loaded from the ig_graph_slam_config file
 */
struct Params {

  /***
   * Params constructor. Fills all variables in Params using config file and
   * wave parser
   */
  Params();

  /***
   * Output params to terminal
   */
  void outputParams();

  /***
   * Get the config file for the matcher. It also checks that the config file
   * can be found and a valid matcher type was entered.
   * @return
   */
  std::string getMatcherConfig();

  /***
   * Test to make sure the config file has all valid parameters entered by the
   * user
   */
  bool validateParams();

  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW -> what does this do?
  std::string bag_file_path, lidar_topic_loc, lidar_topic_map, gps_topic,
      prev_poses_path, imu_topic, odom_topic, gps_type, matcher_type,
      output_path, config_file_path;
  int knn, set_min_neighbours, iterations, init_method, int_map_size,
      downsample_output_method;
  float trajectory_sampling_dist, map_sampling_dist,
      trajectory_rotation_change, map_rotation_change, distance_match_limit,
      distance_match_min, rotation_match_limit, rotation_match_min,
      input_downsample_size, downsample_cell_size, set_search_radius,
      x_lower_threshold, x_upper_threshold,
      y_lower_threshold, y_upper_threshold,
      z_lower_threshold, z_upper_threshold,
      x_lower_threshold_map, x_upper_threshold_map,
      y_lower_threshold_map, y_upper_threshold_map,
      z_lower_threshold_map, z_upper_threshold_map,
      loop_max_distance, loop_min_travel_distance;

  bool ground_segment, combine_scans, use_gps, visualize, downsample_input,
      step_matches, optimize_gps_lidar, fixed_scan_transform_cov,
      use_prev_poses, use_rad_filter, use_pass_through_filter,
      use_pass_through_filter_map;
  Eigen::Affine3d T_LIDAR_GPS, T_LMAP_LLOC;
  wave::MatX scan_transform_cov;
  std::vector<std::string> topics;

};


#endif //IG_GRAPH_SLAM_SLAM_PARAMS_HPP
