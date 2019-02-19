// ROS Headers
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <novatel_msgs/INSPVAX.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

// WAVE Headers
#include <wave/containers/measurement_container.hpp>
#include <wave/utils/log.hpp>

// IG Graph SLAM Headers
#include "conversions.hpp"
#include "load_ros_data.hpp"
#include "measurementtypes.hpp"
#include "pcl_filters.hpp"
#include "scan_matcher.hpp"
#include "slam_params.hpp"
#include "utils.hpp"

#define DEG_TO_RAD 0.0174532925199433
#define RAD_TO_DEG 57.2957795130823209

int gpsMsgCount = 0, imuMsgCount = 0, odomMsgCount = 0, pointCloudMsgCount = 0,
    pointCloudMsgCountMap = 0;
double initial_heading = 0;

Eigen::Affine3d ROSBag::getGPSTransform(const TimePoint &time_point,
                                        bool applyT_ENU_GPS) {
  return gpsToEigen(this->gps_container.get(time_point, 0).first,
                    applyT_ENU_GPS);
}

TimePoint ROSBag::getLidarScanTimePoint(int index) {
  return this->lidar_container[index].time_point;
}

Eigen::Affine3d gpsToEigen(const Eigen::Matrix<double, 6, 1> measurement,
                           bool applyT_ENU_GPS) {
  double T_ECEF_ENU[4][4];
  wave_spatial_utils::ecefFromENUTransformMatrix(measurement.data(), T_ECEF_ENU,
                                                 true);

  // Transformation from the ENU frame formed at the position of the GPS
  // antenna to ECEF
  Eigen::Affine3d E_T_ECEF_ENU;
  E_T_ECEF_ENU.matrix() << T_ECEF_ENU[0][0], T_ECEF_ENU[0][1], T_ECEF_ENU[0][2],
      T_ECEF_ENU[0][3], T_ECEF_ENU[1][0], T_ECEF_ENU[1][1], T_ECEF_ENU[1][2],
      T_ECEF_ENU[1][3], T_ECEF_ENU[2][0], T_ECEF_ENU[2][1], T_ECEF_ENU[2][2],
      T_ECEF_ENU[2][3], T_ECEF_ENU[3][0], T_ECEF_ENU[3][1], T_ECEF_ENU[3][2],
      T_ECEF_ENU[3][3];

  // outputTransform(E_T_ECEF_ENU, "E_T_ECEF_ENU");

  Eigen::Affine3d T_ENU_GPSIMU;

  double roll = measurement(3);
  double pitch = measurement(4);
  double yaw = measurement(5);

  double c_phi = cos(roll);
  double s_phi = sin(roll);
  double c_theta = cos(pitch);
  double s_theta = sin(pitch);
  double c_psi = cos(yaw);
  double s_psi = sin(yaw);

  T_ENU_GPSIMU.matrix()(0, 0) = c_psi * c_phi - s_psi * s_theta * s_phi;
  T_ENU_GPSIMU.matrix()(0, 1) = -s_psi * c_theta;
  T_ENU_GPSIMU.matrix()(0, 2) = c_psi * s_phi + s_psi * s_theta * c_phi;
  T_ENU_GPSIMU.matrix()(1, 0) = s_psi * c_phi + c_psi * s_theta * s_phi;
  T_ENU_GPSIMU.matrix()(1, 1) = c_psi * c_theta;
  T_ENU_GPSIMU.matrix()(1, 2) = s_psi * s_phi - c_psi * s_theta * c_phi;
  T_ENU_GPSIMU.matrix()(2, 0) = -c_theta * s_phi;
  T_ENU_GPSIMU.matrix()(2, 1) = s_theta;
  T_ENU_GPSIMU.matrix()(2, 2) = c_theta * c_phi;
  T_ENU_GPSIMU.matrix()(0, 3) = 0.0;
  T_ENU_GPSIMU.matrix()(1, 3) = 0.0;
  T_ENU_GPSIMU.matrix()(2, 3) = 0.0;
  T_ENU_GPSIMU.matrix()(3, 3) = 1.0;

  // outputTransform(T_ENU_GPSIMU, "T_ENU_GPSIMU");
  // outputTransform(E_T_ECEF_ENU * T_ENU_GPSIMU, "T_ECEF_GPSIMU");
  Eigen::Affine3d T_ECEF_GPSIMU;
  if (applyT_ENU_GPS) {
    return E_T_ECEF_ENU * T_ENU_GPSIMU;
  } else {
    return E_T_ECEF_ENU;
  }
}

ROSBag::ROSBag(Params &p_) {
  this->params = p_;
  this->T_ECEF_MAP.setIdentity();
  this->have_GPS_datum = false;
  this->pcl_pc2_tmp = boost::make_shared<pcl::PCLPointCloud2>();
  this->cloud_tmp = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

// Messages needed for IG
void ROSBag::loadGPSDataFromNavSatFix(
    boost::shared_ptr<sensor_msgs::NavSatFix> gps_msg) {
  try {
    // get rpy from imu container:
    TimePoint imu_msg_time = rosTimeToChrono(gps_msg->header);
    // std::cout << "TimePoint" << imu_msg_time.time_since_epoch().count()<<
    // std::endl;
    auto rpy_msg =
        this->imu_container.get(imu_msg_time, 1); // (vector6, vector6)
    wave::Vec6 gps_measurement;                   // LLA, RPY
    gps_measurement << gps_msg->latitude, gps_msg->longitude, gps_msg->altitude,
        rpy_msg.first(0), // roll
        rpy_msg.first(1), // pitch
        rpy_msg.first(2); // yaw

    wave::Vec6 gps_stdev;
    gps_stdev << sqrt(gps_msg->position_covariance[0]),
        sqrt(gps_msg->position_covariance[4]),
        sqrt(gps_msg->position_covariance[8]),
        rpy_msg.second(0), // roll stddev
        rpy_msg.second(1), // pitch stddev
        rpy_msg.second(2); // yaw stddev

    // This sets your initial transform from map frame to earth-centered
    // earth-fixed frame
    if (!this->have_GPS_datum) {
      this->have_GPS_datum = true;
      this->T_ECEF_MAP = gpsToEigen(gps_measurement, false);
    }
    this->gps_container.emplace(
        rosTimeToChrono(gps_msg->header), 1,
        std::make_pair(gps_measurement, gps_stdev)); // (Vector6, Vector6)
  } catch (const std::out_of_range &e) {
    LOG_INFO("No IMU orientation for time of gps measurement, may happen at "
             "edges of recorded data.");
  }
}

void ROSBag::loadOdomDataFromNavMsgsOdometry(
    boost::shared_ptr<nav_msgs::Odometry> odom_msg) {
  // get quaternion from odom msg:
  Eigen::Quaterniond q(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

  // Get transform
  // TODO: Add calibrations - this is technically T_MAP_BaseLink
  Eigen::Matrix3d R_MAP_LIDAR = q.toRotationMatrix();
  Eigen::Vector4d t_MAP_LIDAR;
  t_MAP_LIDAR << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z, 1;

  wave::Mat4 T_MAP_LIDAR;
  T_MAP_LIDAR.setIdentity();
  T_MAP_LIDAR.block<3, 3>(0, 0) = R_MAP_LIDAR;
  T_MAP_LIDAR.rightCols<1>() = t_MAP_LIDAR;

  // Get covariances
  wave::Vec6 odom_stdev;
  odom_stdev << sqrt(odom_msg->pose.covariance[0]),
      sqrt(odom_msg->pose.covariance[7]), sqrt(odom_msg->pose.covariance[14]),
      sqrt(odom_msg->pose.covariance[21]), sqrt(odom_msg->pose.covariance[28]),
      sqrt(odom_msg->pose.covariance[35]);

  // Add to measurement container
  this->odom_container.emplace(
      rosTimeToChrono(odom_msg->header), 2,
      std::make_pair(T_MAP_LIDAR, odom_stdev)); // (Mat4, Vec6)
}

void ROSBag::loadIMUMessage(rosbag::View::iterator &rosbag_iter,
                            bool end_of_bag, bool start_of_bag) {
  if (rosbag_iter->getTopic() == this->params.imu_topic) {
    imuMsgCount++;
    auto imu_msg = rosbag_iter->instantiate<geometry_msgs::Vector3Stamped>();

    if (start_of_bag) {
      initial_heading = -imu_msg->vector.z;
    }

    wave::Vec6 rpy_measurement;
    rpy_measurement << -imu_msg->vector.y,    // roll
        imu_msg->vector.x - 1.570796327,      // pitch (starts at pi/2)
        -imu_msg->vector.z - initial_heading, // yaw
        0,                                    // cannot use Vec3 in libwave for
        0,                                    // measurement containters
        0;
    wave::Vec6 rpy_stdev;
    rpy_stdev << 10, // TODO: get real std dev from imu data
        10, 100, 0, 0, 0;
    this->imu_container.emplace(
        rosTimeToChrono(imu_msg->header), 1,
        std::make_pair(rpy_measurement, rpy_stdev)); // (Vector3, Vector3)
  }
  if (end_of_bag) {
    LOG_INFO("Saved %d IMU Messages.", imuMsgCount);
  }
}

void ROSBag::loadIMUMessagesAll() {

  if (this->params.imu_topic == "") {
    return;
  }

  // Read bag file and create view
  rosbag::Bag bag;
  try {
    bag.open(this->params.bag_file_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException &ex) {
    LOG_ERROR("Bag exception : %s", ex.what());
  }
  rosbag::View view(bag, rosbag::TopicQuery(this->params.topics), ros::TIME_MIN,
                    ros::TIME_MAX, true);

  int bag_counter = 0, total_messages = view.size();
  bool end_of_bag = false, start_of_bag = true;
  if (total_messages == 0) {
    LOG_ERROR("No messages read. Check your topics in config file.");
  }

  for (auto iter = view.begin(); iter != view.end(); iter++) {
    bag_counter++;
    outputPercentComplete(bag_counter, total_messages,
                          "Loading IMU messages...");
    if (bag_counter == total_messages) {
      end_of_bag = true;
    }
    loadIMUMessage(iter, end_of_bag, start_of_bag);
    start_of_bag = false;
  }
  bag.close();
}

void ROSBag::loadROSBagMessage(rosbag::View::iterator &rosbag_iter,
                               bool end_of_bag) {
  if (rosbag_iter->getTopic() == this->params.gps_topic) {
    gpsMsgCount++;
    auto gps_msg_navsatfix = rosbag_iter->instantiate<sensor_msgs::NavSatFix>();
    auto gps_msg_inspvax = rosbag_iter->instantiate<novatel_msgs::INSPVAX>();
    if (this->params.gps_type == "NavSatFix") {
      this->loadGPSDataFromNavSatFix(gps_msg_navsatfix);
    } else if (this->params.gps_type == "INSPVAX") {
      this->loadGPSDataFromINSPVAX(gps_msg_inspvax);
    } else {
      LOG_ERROR("Improper gps_type entered in config file. input: %s. Use "
                "NavSatFix or INSPVAX.",
                this->params.gps_type);
    }
  } else if (rosbag_iter->getTopic() == this->params.lidar_topic_loc) {
    pointCloudMsgCount++;
    auto lidar_msg = rosbag_iter->instantiate<sensor_msgs::PointCloud2>();
    this->loadPCLPointCloudFromPointCloud2(lidar_msg);
  } else if (rosbag_iter->getTopic() == this->params.lidar_topic_map) {
    pointCloudMsgCountMap++;
    auto lidar_msg = rosbag_iter->instantiate<sensor_msgs::PointCloud2>();
    this->loadPCLPointCloudFromPointCloud2Map(lidar_msg);
  } else if (rosbag_iter->getTopic() == this->params.odom_topic) {
    odomMsgCount++;
    auto odom_msg = rosbag_iter->instantiate<nav_msgs::Odometry>();
    this->loadOdomDataFromNavMsgsOdometry(odom_msg);
  }
  if (end_of_bag) {
    LOG_INFO("Saved %d GPS Messages.", gpsMsgCount);
    LOG_INFO("Saved %d Point Cloud Messages For Localization.",
             pointCloudMsgCount);
    LOG_INFO("Saved %d Point Cloud Messages For Mapping.",
             pointCloudMsgCountMap);
    LOG_INFO("Saved %d Odometry Messages.", odomMsgCount);
  }
}

void ROSBag::loadROSBagMessagesAll() {

  // Read bag file and create view
  rosbag::Bag bag;
  try {
    bag.open(this->params.bag_file_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException &ex) {
    LOG_ERROR("Bag exception : %s", ex.what());
  }
  rosbag::View view(bag, rosbag::TopicQuery(this->params.topics), ros::TIME_MIN,
                    ros::TIME_MAX, true);

  int bag_counter = 0, total_messages = view.size();
  bool end_of_bag = false;

  for (auto iter = view.begin(); iter != view.end(); iter++) {
    bag_counter++;
    if (bag_counter == total_messages) {
      end_of_bag = true;
    }
    outputPercentComplete(bag_counter, total_messages,
                          "Loading all other messages...");
    loadROSBagMessage(iter, end_of_bag);
  }
  bag.close();
}

void ROSBag::loadPCLPointCloudFromPointCloud2(
    boost::shared_ptr<sensor_msgs::PointCloud2> lidar_msg) {
  pcl_conversions::toPCL(*lidar_msg, *this->pcl_pc2_tmp);
  pcl::fromPCLPointCloud2(*this->pcl_pc2_tmp, *this->cloud_tmp);

  // check this, it might not be implemented correctly! See how CropXY is used
  if (this->params.use_pass_through_filter) {
    wave::Vec6 threshold_vec;
    threshold_vec << this->params.x_lower_threshold,
        this->params.x_upper_threshold, this->params.y_lower_threshold,
        this->params.y_upper_threshold, this->params.z_lower_threshold,
        this->params.z_upper_threshold;

    *this->cloud_tmp = passThroughFilterIG(this->cloud_tmp, threshold_vec);
  }

  // ***Add ground segmenter here?

  if (this->params.downsample_input) {
    *this->cloud_tmp =
        downSampleFilterIG(this->cloud_tmp, this->params.input_downsample_size);
  }

  if (this->params.use_rad_filter) {
    *this->cloud_tmp =
        radFilterIG(this->cloud_tmp, this->params.set_min_neighbours,
                    this->params.set_search_radius);
  }

  wave::PCLPointCloudPtr temp =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *temp = *this->cloud_tmp;
  this->lidar_container.emplace_back(rosTimeToChrono(lidar_msg->header), 0,
                                     temp);
}

void ROSBag::loadPCLPointCloudFromPointCloud2Map(
    boost::shared_ptr<sensor_msgs::PointCloud2> lidar_msg) {
  pcl_conversions::toPCL(*lidar_msg, *this->pcl_pc2_tmp);
  pcl::fromPCLPointCloud2(*this->pcl_pc2_tmp, *this->cloud_tmp);

  // check this, it might not be implemented correctly! See how CropXY is used
  if (this->params.use_pass_through_filter) {
    wave::Vec6 threshold_vec;
    threshold_vec << this->params.x_lower_threshold_map,
        this->params.x_upper_threshold_map, this->params.y_lower_threshold_map,
        this->params.y_upper_threshold_map, this->params.z_lower_threshold_map,
        this->params.z_upper_threshold_map;

    *this->cloud_tmp = passThroughFilterIG(this->cloud_tmp, threshold_vec);
  }

  // ***Add ground segmenter here?

  if (this->params.downsample_input) {
    *this->cloud_tmp =
        downSampleFilterIG(this->cloud_tmp, this->params.input_downsample_size);
  }

  if (this->params.use_rad_filter) {
    *this->cloud_tmp =
        radFilterIG(this->cloud_tmp, this->params.set_min_neighbours,
                    this->params.set_search_radius);
  }

  wave::PCLPointCloudPtr temp =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *temp = *this->cloud_tmp;
  this->lidar_container_map.emplace_back(rosTimeToChrono(lidar_msg->header), 0,
                                         temp);
}

uint64_t ROSBag::getLidarTimeWindow(const TimePoint T1) {
  uint64_t start_index_ = 0;
  for (size_t i = 0; i < this->lidar_container.size(); i++) {
    if (this->lidar_container[i].time_point == T1) {
      start_index_ = i;
      break;
    } else if (this->lidar_container[i].time_point > T1) {
      start_index_ = i;
      break;
    } else if (i == this->lidar_container.size()) {
      LOG_ERROR("getLidarTimeWindow: Cannot find lidar time window. Time "
                "inputs invalid.");
      return 0;
    }
  }
  return start_index_;
}

// Messages specific for Moose
void ROSBag::loadGPSDataFromINSPVAX(
    boost::shared_ptr<novatel_msgs::INSPVAX> gps_msg) {
  wave::Vec6 gps_measurement; // LLA, RPY
  gps_measurement << gps_msg->latitude, gps_msg->longitude,
      gps_msg->altitude + gps_msg->undulation, gps_msg->roll * DEG_TO_RAD,
      gps_msg->pitch * DEG_TO_RAD, -gps_msg->azimuth * DEG_TO_RAD;
  wave::Vec6 gps_stdev;
  gps_stdev << gps_msg->latitude_std, gps_msg->longitude_std,
      gps_msg->altitude_std, gps_msg->roll_std * DEG_TO_RAD,
      gps_msg->pitch_std * DEG_TO_RAD, gps_msg->azimuth_std * DEG_TO_RAD;

  if (!this->have_GPS_datum) {
    this->have_GPS_datum = true;
    this->T_ECEF_MAP = gpsToEigen(gps_measurement, false);
  }

  this->gps_container.emplace(
      gpsTimeToChrono(gps_msg->header.gps_week,
                      gps_msg->header.gps_week_seconds),
      0, std::make_pair(gps_measurement, gps_stdev)); // (Vector6, Vector6)
}
