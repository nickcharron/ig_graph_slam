#ifndef IG_GRAPH_SLAM_CONVERSIONS_HPP
#define IG_GRAPH_SLAM_CONVERSIONS_HPP

// #include <iostream>
#include <chrono>
// #include <ctime>
//#include <wave/containers/measurement_container.hpp>
//#include <wave/utils/math.hpp>
//#include <wave_spatial_utils/world_frame_conversions.hpp>
//#include <ros/ros.h>
//#include <GeographicLib/Geocentric.hpp>
//#include <GeographicLib/LocalCartesian.hpp>

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

void outputTransform(Eigen::Affine3d T, std::string name);

void ecefPointFromLLH(const double llh[3], double ecef[3]);

TimePoint rosTimeToChrono(const std_msgs::Header &hdr);

inline Eigen::AngleAxisd enu_to_ecef_rotation(const double *lla);

//inline Eigen::Affine3d gpsToEigen(const Eigen::Matrix<double, 6, 1> measurement, bool applyT_ENU_GPS);

TimePoint gpsTimeToChrono(const uint32_t gps_week, const uint32_t gps_milliseconds);

#endif  // IG_GRAPH_SLAM_CONVERSIONS_HPP
