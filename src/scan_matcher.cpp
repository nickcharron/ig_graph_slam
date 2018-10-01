// ROS Headers
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

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
//#include "conversions.hpp"
//#include "measurementtypes.hpp"
//#include "kdtreetype.hpp"

void fillparams(Params &params)
{
    wave::ConfigParser parser;
    parser.addParam("bag_file_path", &(params.bag_file_path));
    parser.addParam("lidar_topic", &(params.lidar_topic));
    parser.addParam("gps_topic", &(params.gps_topic));
    parser.addParam("k_nearest_neighbours", &(params.knn));
    parser.addParam("trajectory_sampling_distance",
                    &(params.trajectory_sampling_dist));
    parser.addParam("max_range", &(params.max_range));
    parser.addParam("z_lower_threshold", &(params.z_lower_threshold));
    parser.addParam("z_upper_threshold", &(params.z_upper_threshold));
    parser.addParam("use_z_filter", &(params.use_z_filter));
    parser.addParam("matcher_type", &(params.matcher_type));
    parser.addParam("matcher_config_path", &(params.matcher_config));
    parser.addParam("ground_segment", &(params.ground_segment));
    parser.addParam("distance_match_min", &(params.distance_match_min));
    parser.addParam("distance_match_limit", &(params.distance_match_limit));
    parser.addParam("use_gps", &(params.use_gps));
    parser.addParam("downsample_cell_size", &(params.downsample_cell_size));
    parser.addParam("iterations", &(params.iterations));
    parser.addParam("visualize", &(params.visualize));
    parser.addParam("downsample_input", &(params.downsample_input));
    parser.addParam("input_downsample_size", &(params.input_downsample_size));
    parser.addParam("step_matches", &(params.step_matches));
    parser.addParam("optimize_gps_lidar", &(params.optimize_gps_lidar));
    parser.addParam("fixed_scan_transform_cov", &(params.fixed_scan_transform_cov));
    parser.addParam("scan_transform_cov", &(params.scan_transform_cov));

    parser.addParam("T_LIDAR_GPS", &(params.T_LIDAR_GPS.matrix()));

    parser.load("config/ig_graph_slam_config.yaml");
    ++params.knn;  // Nearest neighbour search returns same point, so increment
    params.max_range_neg = -params.max_range;
    // might as well square these now
    params.distance_match_limit *= params.distance_match_limit;
    params.distance_match_min *= params.distance_match_min;
}
