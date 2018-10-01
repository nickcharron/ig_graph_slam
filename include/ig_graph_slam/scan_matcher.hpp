#ifndef IG_GRAPH_SLAM_SCAN_MATCHER_HPP
#define IG_GRAPH_SLAM_SCAN_MATCHER_HPP

#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <wave/containers/measurement_container.hpp>
#include <wave/containers/measurement.hpp>
#include <wave/utils/math.hpp>
#include <wave/utils/config.hpp>
#include <wave/utils/log.hpp>
#include <wave/matching/icp.hpp>
#include <wave/matching/gicp.hpp>
#include <wave/matching/pointcloud_display.hpp>
#include <wave/matching/ground_segmentation.hpp>

//#include "kdtreetype.hpp"
#include "gtsam_graph.hpp"

/***
 * Params loaded from the ig_graph_slam_config file
 */
struct Params
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string bag_file_path, lidar_topic, gps_topic, matcher_type, matcher_config;
    int knn, iterations;
    float trajectory_sampling_dist, max_range, max_range_neg, z_lower_threshold,
            z_upper_threshold, distance_match_min, distance_match_limit,
            downsample_cell_size, input_downsample_size;
    bool ground_segment, use_gps, visualize, downsample_input, step_matches,
            optimize_gps_lidar, fixed_scan_transform_cov, use_z_filter;
    Eigen::Affine3d T_LIDAR_GPS;
    wave::MatX scan_transform_cov;
    std::vector<std::string> topics;
  };

  /***
   * Fill params from YAML file into params object
   * @param params
   */
  void fillparams(Params &params);


#endif //IG_GRAPH_SLAM_SCAN_MATCHER_HPP
