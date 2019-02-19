#ifndef IG_GRAPH_SLAM_LOAD_ROS_DATA_HPP
#define IG_GRAPH_SLAM_LOAD_ROS_DATA_HPP

// ROS specific headers
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <novatel_msgs/INSPVAX.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// WAVE specific headers
#include <wave/containers/measurement.hpp>
#include <wave/containers/measurement_container.hpp>
#include <wave/utils/log.hpp>

// IG Graph SLAM specific headers
#include "pcl_filters.hpp"
#include "scan_matcher.hpp"
#include "slam_params.hpp"

// Declare some templates:
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

// declare these due to circular reference
struct ScanMatcher;
//struct Params;

Eigen::Affine3d gpsToEigen(const Eigen::Matrix<double, 6, 1> measurement,
                           bool applyT_ENU_GPS);

struct ROSBag {
  /***
   * ROSBag parent class
  */

  ROSBag(Params &p_);
  //~ROSBag();

  /***
   * Get TimePoint object of the LIDAR scan at the specified index. Needed for
   * decoupling matcher from GTSAM stuff
   * @param index index in lidar container
   * @return
   */
  TimePoint getLidarScanTimePoint(int index);

  /***
   * Gets the GPS Transform at a certain time point
   * @param time_point
   * @param applyT_ENU_GPS
   * @return
   */
  Eigen::Affine3d getGPSTransform(const TimePoint &time_point,
                                  bool applyT_ENU_GPS);

  /***
   * Loads IMU RPY data into container from a geometry_msgs/Vector3Stamped ROS
   * msg
   */
  void loadIMUMessagesAll();

  /***
   * Loads IMU RPY data into container from a geometry_msgs/Vector3Stamped ROS
   * msg
   * @param rosbag_iter
   * @param end_of_bag
   * @param start_of_bag
   */
  void loadIMUMessage(rosbag::View::iterator &rosbag_iter,
                              bool end_of_bag, bool start_of_bag);
  /***
   * Loads GPS data into measurement container from an NavSatFix ROS msg
   * @param gps_msg NavSatFix ROS msg
   */
  void
  loadGPSDataFromNavSatFix(boost::shared_ptr<sensor_msgs::NavSatFix> gps_msg);

  /***
   * Loads Odometry data into measurement container from a Odometry ROS msg
   * @param odom_msg Odometry ROS msg
   */
  void loadOdomDataFromNavMsgsOdometry(
      boost::shared_ptr<nav_msgs::Odometry> odom_msg);

  /***
   * Loads GPS data into measurement container from an INSPVAX ROS msg
   * @param gps_msg INSPVAX ROS msg
   */
  void loadGPSDataFromINSPVAX(boost::shared_ptr<novatel_msgs::INSPVAX> gps_msg);

  /***
   * Load ROS Bag message into their appropriate container
   * @param rosbag_iter
   * @param end_of_bag
   * @param start_of_bag
   */
  void loadROSBagMessage(rosbag::View::iterator &rosbag_iter, bool end_of_bag);

 /***
  * Load all ROS Bag messages into their appropriate containers
  */
  void loadROSBagMessagesAll();

  /***
   * Load PCL point cloud message from ROS message into container (for
   * localization)
   * @param lidar_msg
   */
  void loadPCLPointCloudFromPointCloud2(
      boost::shared_ptr<sensor_msgs::PointCloud2> lidar_msg);

  /***
   * Load PCL point cloud message from ROS message into container (for mapping)
   * @param lidar_msg
   */
  void loadPCLPointCloudFromPointCloud2Map(
      boost::shared_ptr<sensor_msgs::PointCloud2> lidar_msg);

  /***
   * Extract the index within lidar_container which corresponds to a specific
   * time point.
   * @param T1 time_point at which you want the lidar container index
   * @return
   */
  uint64_t getLidarTimeWindow(const TimePoint T1);

  // declare containers
  wave::MeasurementContainer<
      wave::Measurement<std::pair<wave::Vec6, wave::Vec6>, uint>>
      gps_container;
  wave::MeasurementContainer<
      wave::Measurement<std::pair<wave::Vec6, wave::Vec6>, uint>>
      imu_container;
  wave::MeasurementContainer<
      wave::Measurement<std::pair<wave::Mat4, wave::Vec6>, uint>>
      odom_container;
  std::vector<wave::Measurement<wave::PCLPointCloudPtr, uint>> lidar_container;
  std::vector<wave::Measurement<wave::PCLPointCloudPtr, uint>>
      lidar_container_map;

  // declare point cloud objects
  pcl::PCLPointCloud2::Ptr pcl_pc2_tmp;
  wave::PCLPointCloudPtr cloud_tmp;

  // Other required variables
  Params params;
  bool have_GPS_datum;
  Eigen::Affine3d T_ECEF_MAP; // this is in both ROSBag and ScanMatcher structs
};

#endif // IG_GRAPH_SLAM_LOAD_ROS_DATA_HPP
