#ifndef IG_GRAPH_SLAM_CONVERSIONS_HPP
#define IG_GRAPH_SLAM_CONVERSIONS_HPP

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <chrono>
#include <cmath>
#include <ctime>
#include <wave_spatial_utils/world_frame_conversions.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#define DEG_TO_RAD 0.0174532925199433
#define RAD_TO_DEG 57.2957795130823209

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

const uint32_t WEEK_TO_SEC = 604800;
const uint32_t EPOCH_OFFSET = 315964800;
const uint32_t LEAP_SECONDS = 18;

inline std::string
convertTimeToDate(std::chrono::system_clock::time_point time_) {
  using namespace std;
  using namespace std::chrono;
  system_clock::duration tp = time_.time_since_epoch();
  time_t tt = system_clock::to_time_t(time_);
  tm local_tm = *localtime(&tt);

  string outputTime =
      to_string(local_tm.tm_year + 1900) + "_" +
      to_string(local_tm.tm_mon + 1) + "_" + to_string(local_tm.tm_mday) + "_" +
      to_string(local_tm.tm_hour) + "_" + to_string(local_tm.tm_min) + "_" +
      to_string(local_tm.tm_sec);
  return outputTime;
}

inline void ecefPointFromLLH(const double llh[3], double ecef[3]) {
  double latitude = llh[0], longitude = llh[1], height = llh[2];

  GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

  double X, Y, Z;
  earth.Forward(latitude * RAD_TO_DEG, longitude * RAD_TO_DEG, height, X, Y, Z);

  ecef[0] = X;
  ecef[1] = Y;
  ecef[2] = Z;
}

inline TimePoint rosTimeToChrono(const std_msgs::Header &hdr) {
  std::chrono::seconds secs(hdr.stamp.sec);
  std::chrono::nanoseconds nsecs(hdr.stamp.nsec);
  auto dur = secs + nsecs;
  return TimePoint(dur);
}

inline ros::Time chronoToRosTime(const TimePoint &time_point) {
  uint32_t seconds, nanoseconds;
  seconds = std::round(time_point.time_since_epoch().count()/1000000000);
  double tmp = time_point.time_since_epoch().count() -
               std::round(time_point.time_since_epoch().count());
  nanoseconds = std::round(tmp*1000000000);
  ros::Time ros_time(seconds, nanoseconds);
  return ros_time;
}

inline Eigen::AngleAxisd enu_to_ecef_rotation(const double *lla) {
  Eigen::AngleAxisd retval;
  Eigen::Matrix3d rot;
  rot(0, 0) = -sin(lla[1]);
  rot(1, 0) = cos(lla[1]);
  rot(2, 0) = 0;
  rot(0, 1) = -sin(lla[0]) * cos(lla[1]);
  rot(1, 1) = -sin(lla[0]) * sin(lla[1]);
  rot(2, 1) = cos(lla[0]);
  rot(0, 2) = cos(lla[0]) * cos(lla[1]);
  rot(1, 2) = cos(lla[0]) * sin(lla[1]);
  rot(2, 2) = sin(lla[0]);
  retval = rot;
  return retval;
}

inline TimePoint gpsTimeToChrono(const uint32_t gps_week,
                                 const uint32_t gps_milliseconds) {
  uint64_t secs = WEEK_TO_SEC * gps_week + (gps_milliseconds / 1000) +
                  EPOCH_OFFSET - LEAP_SECONDS;
  uint64_t nsecs = (gps_milliseconds % 1000) * 1000000;
  std::chrono::seconds Secs(secs);
  std::chrono::nanoseconds Nsecs(nsecs);
  auto dur = Secs + Nsecs;
  return TimePoint(dur);
}

inline Eigen::Vector3d invSkewTransform(const Eigen::Matrix3d M_){
  Eigen::Vector3d V_;
  V_(0) = M_(2,1);
  V_(1) = M_(0,2);
  V_(2) = M_(1,0);
  return V_;
}

inline Eigen::Matrix3d skewTransform(const Eigen::Vector3d V_){
  Eigen::Matrix3d M_;
  M_(0,0) = 0;
  M_(0,1) = -V_(2,0);
  M_(0,2) = V_(1,0);
  M_(1,0) = V_(2,0);
  M_(1,1) = 0;
  M_(1,2) = -V_(0,0);
  M_(2,0) = -V_(1,0);
  M_(2,1) = V_(0,0);
  M_(2,2) = 0;
  return M_;
}

inline Eigen::Vector3d RToLieAlgebra(const Eigen::Matrix3d R){
  return invSkewTransform(R.log());
}

inline Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d eps){
  return skewTransform(eps).exp();
}

#endif // IG_GRAPH_SLAM_CONVERSIONS_HPP
