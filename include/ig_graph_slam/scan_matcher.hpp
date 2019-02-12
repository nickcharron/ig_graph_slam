#ifndef IG_GRAPH_SLAM_SCAN_MATCHER_HPP
#define IG_GRAPH_SLAM_SCAN_MATCHER_HPP

// PCL headers and other
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unsupported/Eigen/MatrixFunctions>
// WAVE headers
#include "wave/matching/gicp.hpp"
#include <wave/matching/icp.hpp>
#include <wave/matching/pointcloud_display.hpp>
#include <wave/utils/log.hpp>

// IG Graph SLAM headers
#include "gtsam_graph.hpp"
#include "kdtreetype.hpp"
#include "load_ros_data.hpp"

// Declare some templates:
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

/***
 * Params loaded from the ig_graph_slam_config file
 */
struct Params {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string bag_file_path, lidar_topic_loc, lidar_topic_map, gps_topic,
      prev_poses_path, imu_topic, odom_topic, gps_type, matcher_type,
      output_path;
  int knn, set_min_neighbours, iterations, init_method, int_map_size,
      downsample_output_method;
  float trajectory_sampling_dist, map_sampling_dist, distance_match_limit,
      distance_match_min, input_downsample_size, downsample_cell_size,
      set_search_radius, x_lower_threshold, x_upper_threshold,
      y_lower_threshold, y_upper_threshold, z_lower_threshold,
      z_upper_threshold, x_lower_threshold_map, x_upper_threshold_map,
      y_lower_threshold_map, y_upper_threshold_map, z_lower_threshold_map,
      z_upper_threshold_map, loop_max_distance, loop_min_travel_distance;

  bool ground_segment, combine_scans, use_gps, visualize, downsample_input,
      step_matches, optimize_gps_lidar, fixed_scan_transform_cov,
      use_prev_poses, use_rad_filter, use_pass_through_filter,
      use_pass_through_filter_map;
  Eigen::Affine3d T_LIDAR_GPS, T_LMAP_LLOC;
  wave::MatX scan_transform_cov;
  std::vector<std::string> topics;
};

/***
 * Fill params from YAML file into params object
 * @param params
 */
void fillparams(Params &params);

void outputParams(boost::shared_ptr<Params> p_);

std::string getMatcherConfig(std::string matcher);

bool validateParams(boost::shared_ptr<Params> p_);

struct ROSBag;

struct ScanMatcher {
  /***
   * Scan matcher parent class
   * @param p_
   */
  //-------------------------------------------------------------------------

  ScanMatcher(Params &p_) : params(p_), init_display("Graph") {
    // Visualization:
    if (this->params.visualize) {
      this->init_display.startSpin();
    }
    this->cloud_temp_display =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->aggregate = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->cloud_target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->pcl_pc2 = boost::make_shared<pcl::PCLPointCloud2>();
    // this->total_matches = 0;
  }
  ~ScanMatcher() {
    if (this->params.visualize) {
      this->init_display.stopSpin();
    }
  }

  /***
   * Creates pose scan map with GPS data
   * @param ros_data
   */
  void createPoseScanMap(boost::shared_ptr<ROSBag> ros_data);

  /***
   * Find Adjacent scans for all scans. Adjacent scans will only consist of
   * future scans with respect to the current scan
   */
  void findAdjacentScans();

  /***
   * Find loop closure opportunities for all pose scans.
   */
  void findLoops();

  /***
   * Matches the scan
   * @param [in] i first scan index in initial poses
   * @param [in] j second scan index in initial poses
   * @param [out] L_Li_Lj Transform of the pose of scan j in the frame of scan i
   * @param [out] info Information matrix associated to resulting transform
   * @param [out] correction_norm_valid bool to check if correction_norm is
   * below a specified threshold
   * @return
   */
  virtual bool matchScans(uint64_t i, uint64_t j, Eigen::Affine3d &L_Li_Lj,
                          wave::Mat6 &info, bool &correction_norm_valid,
                          boost::shared_ptr<ROSBag> ros_data) = 0;

  /***
   * Creates the aggregate map pointcloud. All the pointclouds are downsampled
   * and concatenated into intermediate pointclouds. The intermeditae
   * pointclouds are then downsampled and concatenated into an aggregate
   * pointcloud.
   * @param graph
   * @param ros_data
   */
  void createAggregateMap(GTSAMGraph &graph, boost::shared_ptr<ROSBag> ros_data,
                          int mapping_method);

  /***
   * Outputs the aggregate pointcloud map as a pcd file
   * @param graph
   */
  void outputAggregateMap(GTSAMGraph &graph, boost::shared_ptr<ROSBag> ros_data,
                          int mapping_method, std::string path_);

  void displayPointCloud(
      wave::PCLPointCloudPtr cloud_display, int color,
      const Eigen::Affine3d &transform = Eigen::Affine3d::Identity());

  int total_matches;
  // Eigen::Affine3d T_ECEF_MAP; // TODO: is this needed?
  Params params;
  wave::PointCloudDisplay init_display;
  InitPose<double> init_pose; // see kdtreetype.hpp
  std::vector<int> pose_scan_map;
  std::vector<wave::Measurement<Eigen::Affine3d, uint>> final_poses;
  boost::shared_ptr<std::vector<std::vector<uint64_t>>> adjacency;
  boost::shared_ptr<std::vector<std::vector<uint64_t>>> loops;
  pcl::PCLPointCloud2::Ptr
      pcl_pc2; // used an intermediate when converting from ROS messages
  wave::PCLPointCloudPtr cloud_temp_display, aggregate, cloud_target;
};

class ICPScanMatcher : public ScanMatcher {
public:
  ICPScanMatcher(Params &p_, std::string matcherConfigPath);
  ~ICPScanMatcher() {}

  bool matchScans(uint64_t i, uint64_t j, Eigen::Affine3d &L_Li_Lj,
                  wave::Mat6 &info, bool &correction_norm_valid,
                  boost::shared_ptr<ROSBag> ros_data);

  wave::ICPMatcher matcher;

  // Declared here to avoid multiple allocation/deallocations
  wave::PCLPointCloudPtr cloud_ref,
      cloud_tgt; // used as intermediates to apply filters/matches
};

class GICPScanMatcher : public ScanMatcher {
public:
  GICPScanMatcher(Params &p_, std::string matcherConfigPath);
  ~GICPScanMatcher() {}

  bool matchScans(uint64_t i, uint64_t j, Eigen::Affine3d &L_Li_Lj,
                  wave::Mat6 &info, bool &correction_norm_valid,
                  boost::shared_ptr<ROSBag> ros_data);

  wave::GICPMatcher matcher;

  // Declared here to avoid multiple allocation/deallocations
  wave::PCLPointCloudPtr cloud_ref,
      cloud_tgt; // used as intermediates to apply filters/matches
};

#endif // IG_GRAPH_SLAM_SCAN_MATCHER_HPP
