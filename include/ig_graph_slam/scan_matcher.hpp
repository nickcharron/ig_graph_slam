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
#include "load_ros_data.hpp"
#include "slam_params.hpp"

// Declare some templates:
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

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
  }
  ~ScanMatcher() {
    if (this->params.visualize) {
      this->init_display.stopSpin();
    }
  }

  /***
   * graph builder and optimizer main function
   * @return
   */
  GTSAMGraph buildGTSAMGraph(boost::shared_ptr<ROSBag> ros_data);

  /***
  * Takes the config file for ig_graph_slam and copies it to the output
  * directory to have a copy of the parameters used for each map
  * @param save_path_ output directory
  */
  void saveParamsFile(std::string save_path_);

  /***
  * Plots the gtsam graph and saves it as a .dot file in the output directory
  * @param save_path_ output directory
  * @param graph_ gtsam graph object
  */
  void saveGraphFile(GTSAMGraph graph_, std::string save_path_);

  /***
   * Takes new scan if the distance between the scans are more than dist, or if
   * the change in rotation is more than rot
   * @param p1 pose of first scan
   * @param p2 pose of second scan
   * @param dist minimum distance between scans
   * @param rot minimum rotation change between scans (in deg)
   * @return
   */
  bool takeNewScan(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2,
                   const double &dist, const double &rot);

 /***
  * Load poses from a previous graph
  */
  void loadPrevPoses();

  /***
   * Creates pose scan map based on specified initialization method
   * @param ros_data
   */
  void createPoseScanMap(boost::shared_ptr<ROSBag> ros_data);

  /***
   * Creates pose scan map with inputted poses data
   * @param ros_data
   */
  void createPoseScanMapFromPoses(boost::shared_ptr<ROSBag> ros_data);

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
   * @param mapping_method
   */
  void createAggregateMap(boost::shared_ptr<ROSBag> ros_data,
                          int mapping_method);

  /***
   * Outputs the aggregate pointcloud map as a pcd file
   * @param mapping_method
   * @param path_
   */
  void outputAggregateMap(int mapping_method, std::string path_);

  /***
   * Outputs the optimized trajectory into a text file along with timestamps
   * @param path_
   */
  void outputOptTraj(std::string path_);

  /***
   * Outputs the initial trajectory into a text file along with timestamps
   * @param path_
   */
  void outputInitTraj(std::string path_);

  void displayPointCloud(
      wave::PCLPointCloudPtr cloud_display, int color,
      const Eigen::Affine3d &transform = Eigen::Affine3d::Identity());

  int total_matches;
  Params params;
  wave::PointCloudDisplay init_display;
  std::vector<int> pose_scan_map;
  std::vector<wave::Measurement<Eigen::Affine3d, uint>> init_pose;
  std::vector<wave::Measurement<Eigen::Affine3d, uint>> final_poses;
  boost::shared_ptr<std::vector<std::vector<uint64_t>>> adjacency;
  boost::shared_ptr<std::vector<std::vector<uint64_t>>> loops;
  pcl::PCLPointCloud2::Ptr pcl_pc2;
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
  wave::PCLPointCloudPtr cloud_ref, cloud_tgt;
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
