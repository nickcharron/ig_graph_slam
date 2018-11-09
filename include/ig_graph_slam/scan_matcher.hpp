#ifndef IG_GRAPH_SLAM_SCAN_MATCHER_HPP
#define IG_GRAPH_SLAM_SCAN_MATCHER_HPP

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// WAVE headers
#include <wave/utils/log.hpp>
#include <wave/matching/icp.hpp>
#include <wave/matching/pointcloud_display.hpp>

// IG Graph SLAM headers
#include "load_ros_data.hpp"
#include "kdtreetype.hpp"
#include "gtsam_graph.hpp"

// Declare some templates:
  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;

/***
 * Params loaded from the ig_graph_slam_config file
 */
struct Params
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string bag_file_path, lidar_topic_loc, lidar_topic_map, gps_topic, gps_imu_topic, odom_topic,
    gps_type, matcher_type, matcher_config;
    int knn, iterations, init_method, mapping_method, int_map_size;
    float trajectory_sampling_dist, map_sampling_dist,
          distance_match_limit, distance_match_min,
          input_downsample_size, downsample_cell_size,
          set_search_radius, set_min_neighbours,
          x_lower_threshold, x_upper_threshold,
          y_lower_threshold, y_upper_threshold,
          z_lower_threshold, z_upper_threshold,
          x_lower_threshold_map, x_upper_threshold_map,
          y_lower_threshold_map, y_upper_threshold_map,
          z_lower_threshold_map, z_upper_threshold_map;

    bool ground_segment, use_gps, visualize, downsample_input, step_matches,
            optimize_gps_lidar, fixed_scan_transform_cov,
            use_rad_filter, use_pass_through_filter, use_pass_through_filter_map;
    Eigen::Affine3d T_LIDAR_GPS, T_LMAP_LLOC;
    wave::MatX scan_transform_cov;
    std::vector<std::string> topics;
};

/***
 * Fill params from YAML file into params object
 * @param params
 */
void fillparams(Params &params);

struct ROSBag;

struct ScanMatcher
{
    /***
     * Scan matcher parent class
     * @param p_
     */
     //-------------------------------------------------------------------------

     ScanMatcher(Params &p_): params(p_), init_display("Graph")
     {
       // Visualization:
         if (this->params.visualize)
         {
             this->init_display.startSpin();
         }
         this->cloud_temp_display = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
         //this->total_matches = 0;
     }
     ~ScanMatcher()
     {
         if (this->params.visualize)
         {
             this->init_display.stopSpin();
         }
     }


   /***
    * Creates pose scan map with GPS data
    * @param ros_data
    */
   //virtual void createPoseScanMap() = 0;
   virtual void createPoseScanMap(boost::shared_ptr<ROSBag> ros_data) = 0;

   /***
    * Find Adjacent scans for all scans. Adjacent scans will only consist of
    * future scans with respect to the current scan
    */
   void findAdjacentScans();

   /***
     * Matches the scan
     * @param [in] i first scan index in initial poses
     * @param [in] j second scan index in initial poses
     * @param [out] L_Li_Lj Transform of the pose of scan j in the frame of scan i
     * @param [out] info Information matrix associated to resulting transform
     * @param [out] correction_norm_valid bool to check if correction_norm is below a specified threshold
     * @return
     */
    virtual bool matchScans(uint64_t i, uint64_t j, Eigen::Affine3d &L_Li_Lj, wave::Mat6 &info, bool &correction_norm_valid,  boost::shared_ptr<ROSBag> ros_data) = 0;

   /***
     * Creates the aggregate map pointcloud. All the pointclouds are downsampled
     * and concatenated into intermediate pointclouds. The intermeditae pointclouds
     * are then downsampled and concatenated into an aggregate pointcloud.
     * @param graph
     * @param ros_data
     */
    virtual void createAggregateMap(GTSAMGraph &graph, boost::shared_ptr<ROSBag> ros_data) = 0;

    /***
     * Outputs the aggregate pointcloud map as a pcd file
     * @param graph
     */
    virtual void outputAggregateMap(GTSAMGraph &graph, boost::shared_ptr<ROSBag> ros_data) = 0;

   int total_matches;
   //Eigen::Affine3d T_ECEF_MAP; // TODO: is this needed?
   Params params;
   wave::PointCloudDisplay init_display;
   wave::PCLPointCloudPtr cloud_temp_display;
   InitPose<double> init_pose; // see kdtreetype.hpp
   std::vector<int> pose_scan_map;
   std::vector<wave::Measurement<Eigen::Affine3d, uint>> final_poses;
   boost::shared_ptr<std::vector<std::vector<uint64_t>>> adjacency;

};

class ICP1ScanMatcher : public ScanMatcher
{
  public:
    ICP1ScanMatcher(Params &p_);
    ~ICP1ScanMatcher() {}

    void createPoseScanMap(boost::shared_ptr<ROSBag> ros_data);
    bool matchScans(uint64_t i, uint64_t j, Eigen::Affine3d &L_Li_Lj, wave::Mat6 &info, bool &correction_norm_valid,  boost::shared_ptr<ROSBag> ros_data);
    void displayPointCloud(wave::PCLPointCloudPtr cloud_display, int color, const Eigen::Affine3d &transform = Eigen::Affine3d::Identity());
    void createAggregateMap(GTSAMGraph &graph, boost::shared_ptr<ROSBag> ros_data);
    void outputAggregateMap(GTSAMGraph &graph, boost::shared_ptr<ROSBag> ros_data);

    wave::ICPMatcher matcher;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> intermediaries;
    wave::PCLPointCloudPtr aggregate;

    // Declared here to avoid multiple allocation/deallocations
    pcl::PCLPointCloud2::Ptr pcl_pc2; // used an intermediate when converting from ROS messages
    wave::PCLPointCloudPtr cloud_ref, cloud_target; // used as intermediates to apply filters/matches
};

#endif //IG_GRAPH_SLAM_SCAN_MATCHER_HPP
