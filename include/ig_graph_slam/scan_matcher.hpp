#ifndef IG_GRAPH_SLAM_SCAN_MATCHER_HPP
#define IG_GRAPH_SLAM_SCAN_MATCHER_HPP

#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>

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
    float trajectory_sampling_dist, max_range, max_range_neg,
          input_downsample_size, downsample_cell_size,
          set_search_radius, set_min_neighbours,
          x_lower_threshold, x_upper_threshold,
          y_lower_threshold, y_upper_threshold,
          z_lower_threshold, z_upper_threshold;

    bool ground_segment, use_gps, visualize, downsample_input, step_matches,
            optimize_gps_lidar, fixed_scan_transform_cov,
            use_rad_filter, use_pass_through_filter;
    Eigen::Affine3d T_LIDAR_GPS;
    wave::MatX scan_transform_cov;
    std::vector<std::string> topics;
};

/***
 * Fill params from YAML file into params object
 * @param params
 */
void fillparams(Params &params);

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
         this->T_ECEF_MAP.setIdentity();
         this->have_GPS_datum = false;
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
    * Loads GPS data into scan matcher from an NavSatFix ROS msg
    * @param gps_msg NavSatFix ROS msg
    */
   void loadGPSDataFromNavSatFix(boost::shared_ptr<sensor_msgs::NavSatFix> gps_msg);

   /***
    * Load ROS Bag message into their appropriate container
    * @param rosbag_iter
    */
   virtual void loadROSBagMessage(rosbag::View::iterator &rosbag_iter, bool end_of_bag) = 0;

   bool have_GPS_datum;
   //int total_matches;
   Eigen::Affine3d T_ECEF_MAP;
   Params params;
   wave::PointCloudDisplay init_display;
   wave::PCLPointCloudPtr cloud_temp_display;
   wave::MeasurementContainer<wave::Measurement<std::pair<wave::Vec6, wave::Vec6>, uint>> gps_container;
};

class ICP1ScanMatcher : public ScanMatcher
{
  public:
    ICP1ScanMatcher(Params &p_);
    ~ICP1ScanMatcher() {}
    void loadROSBagMessage(rosbag::View::iterator &rosbag_iter, bool end_of_bag);
    void loadPCLPointCloudFromPointCloud2(boost::shared_ptr<sensor_msgs::PointCloud2> lidar_msg);
    //void loadGPSDataFromNavSatFix(boost::shared_ptr<sensor_msgs::NavSatFix> gps_msg);

    pcl::PassThrough<pcl::PointXYZ> pass_filter_x, pass_filter_y, pass_filter_z;
    pcl::VoxelGrid<pcl::PointXYZ> downsampler;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radfilter;
    wave::ICPMatcher matcher;
    std::vector<wave::Measurement<wave::PCLPointCloudPtr, uint>> lidar_container;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> intermediaries;
    wave::PCLPointCloudPtr aggregate;

    // Declared here to avoid multiple allocation/deallocations
    pcl::PCLPointCloud2::Ptr pcl_pc2; // used an intermediate when converting from ROS messages
    wave::PCLPointCloudPtr cloud_ref, cloud_target; // used as intermediates to apply filters/matches
};

#endif //IG_GRAPH_SLAM_SCAN_MATCHER_HPP
