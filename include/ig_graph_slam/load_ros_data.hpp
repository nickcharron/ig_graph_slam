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

// Declare some templates:
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

/***
 * Takes new scan if the distance between the scans are more than dist
 * @param p1 pose of first scan
 * @param p2 pose of second scan
 * @param dist minimum distance between scans
 * @return
 */
bool takeNewScan(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2,
                 const double &dist);

// declare these due to circular reference
struct ScanMatcher;
struct Params;

Eigen::Affine3d gpsToEigen(const Eigen::Matrix<double, 6, 1> measurement,
                           bool applyT_ENU_GPS);

struct ROSBag {
  /***
   * ROSBag parent class
   */
  //-------------------------------------------------------------------------

  ROSBag();
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
   * @param rosbag_iter
   * @param end_of_bag
   * @param start_of_bag
   * @param imu_topic
   */
  void loadIMUMessage(rosbag::View::iterator &rosbag_iter, bool end_of_bag,
                      bool start_of_bag, std::string imu_topic);

  /***
   * Loads GPS data into measurement container from an NavSatFix ROS msg
   * @param gps_msg NavSatFix ROS msg
   * @param p_
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
   * @param p_
   */
  void loadGPSDataFromINSPVAX(boost::shared_ptr<novatel_msgs::INSPVAX> gps_msg);

  /***
   * Load ROS Bag message into their appropriate container
   * @param rosbag_iter
   * @param end_of_bag
   * @param start_of_bag
   * @param p_
   */
  void loadROSBagMessage(rosbag::View::iterator &rosbag_iter, bool end_of_bag,
                         boost::shared_ptr<Params> p_);

  /***
   * Load PCL point cloud message from ROS message into container (for
   * localization)
   * @param lidar_msg
   * @param p_
   */
  void loadPCLPointCloudFromPointCloud2(
      boost::shared_ptr<sensor_msgs::PointCloud2> lidar_msg,
      boost::shared_ptr<Params> p_);

  /***
   * Load PCL point cloud message from ROS message into container (for mapping)
   * @param lidar_msg
   * @param p_
   */
  void loadPCLPointCloudFromPointCloud2Map(
      boost::shared_ptr<sensor_msgs::PointCloud2> lidar_msg,
      boost::shared_ptr<Params> p_);

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

  // declare filter objects
  pcl::PCLPointCloud2::Ptr
      pcl_pc2_tmp; // used an intermediate when converting from ROS messages
  wave::PCLPointCloudPtr cloud_tmp;

  // Other required variables
  bool have_GPS_datum;
  Eigen::Affine3d T_ECEF_MAP; // this is in both ROSBag and ScanMatcher structs
};

#endif // IG_GRAPH_SLAM_LOAD_ROS_DATA_HPP
