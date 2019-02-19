#ifndef IG_GRAPH_SLAM_MEASUREMENTTYPES_HPP
#define IG_GRAPH_SLAM_MEASUREMENTTYPES_HPP

#include <chrono>
#include <math.h>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <wave/containers/measurement.hpp>
#include <wave/utils/math.hpp>

using GPSMeasurement =
    wave::Measurement<std::pair<wave::Vec6, wave::Vec6>, uint>;
using TransformMeasurement =
    wave::Measurement<std::pair<wave::Mat4, wave::Vec6>, uint>;
using PCLLidarMeasurement = wave::Measurement<wave::PCLPointCloudPtr, uint>;
using LidarContainer =
    std::vector<wave::Measurement<wave::PCLPointCloudPtr, uint>>;

// using LPMLidarMeasurement =
// wave::Measurement<boost::shared_ptr<PointMatcher<double>::DataPoints>, uint>;
using PoseMeasurement = wave::Measurement<Eigen::Affine3d, uint>;
using TimeType = std::chrono::steady_clock::time_point;
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

namespace wave {

inline double wrapToPI(double theta) {
  double retval = theta;
  if (theta > M_PI) {
    retval -= 2 * M_PI;
  } else if (theta < -M_PI) {
    retval += 2 * M_PI;
  }
  return retval;
}

inline decltype(GPSMeasurement::value) interpolate(const GPSMeasurement &m1,
                                                   const GPSMeasurement &m2,
                                                   const TimeType &t) {
  double w2 = 1.0 * (t - m1.time_point) / (m2.time_point - m1.time_point);

  wave::Vec6 first =
      (1 - w2) * std::get<0>(m1.value) + w2 * std::get<0>(m2.value);
  wave::Vec6 second =
      (1 - w2) * std::get<1>(m1.value) + w2 * std::get<1>(m2.value);

  auto yaw1 = std::get<0>(m1.value)(5);
  auto yaw2 = std::get<0>(m2.value)(5);

  double delta = yaw2 - yaw1;
  delta = wrapToPI(delta);

  first(5) = yaw1 + w2 * delta;
  first(5) = wrapToPI(first(5));
  return std::make_pair(first, second);
}

inline decltype(TransformMeasurement::value)
interpolate(const TransformMeasurement &m1, const TransformMeasurement &m2,
            const TimeType &t) {
  double w2 = 1.0 * (t - m1.time_point) / (m2.time_point - m1.time_point);

  wave::Mat4 T1 = std::get<0>(m1.value);
  wave::Mat4 T2 = std::get<0>(m2.value);
  wave::Mat4 T;
  wave::Vec6 std_dev =
      (1 - w2) * std::get<1>(m1.value) + w2 * std::get<1>(m2.value);

  wave::Mat3 R1 = T1.block<3, 3>(0, 0);
  wave::Mat3 R2 = T2.block<3, 3>(0, 0);
  wave::Mat3 R = (R2 * R1.transpose()).pow(w2) * R1;

  wave::Vec4 t1 = T1.rightCols<1>();
  wave::Vec4 t2 = T2.rightCols<1>();
  wave::Vec4 trans = (1 - w2) * t1 + w2 * t2;

  T.setIdentity();
  T.block<3, 3>(0, 0) = R;
  T.rightCols<1>() = trans;

  return std::make_pair(T, std_dev);
}

inline std::pair<uint64_t, uint64_t>
getLidarTimeWindow(const LidarContainer &lidar_container_, const TimePoint T1,
                   const TimePoint T2) {
  uint64_t start_index_ = 0, end_index_ = 0;
  for (size_t i = 0; i < lidar_container_.size(); i++) {
    if (lidar_container_[i].time_point == T1) {
      start_index_ = i;
      break;
    } else if (lidar_container_[i].time_point > T1) {
      start_index_ = i;
      break;
    } else if (i == lidar_container_.size()) {
      LOG_ERROR("getLidarTimeWindow: Cannot find lidar time window. Time "
                "inputs invalid.");
      std::pair<uint64_t, uint64_t> timewindow(0, 0);
      return timewindow;
    }
  }
  for (size_t i = start_index_; i < lidar_container_.size(); i++) {
    if (lidar_container_[i].time_point == T2) {
      end_index_ = i;
      break;
    } else if (lidar_container_[i].time_point > T2) {
      end_index_ = i - 1;
      break;
    } else if (i == lidar_container_.size() - 1) {
      LOG_ERROR("getLidarTimeWindow: Cannot find lidar time window. Time "
                "inputs invalid.");
      std::pair<uint64_t, uint64_t> timewindow(0, 0);
      return timewindow;
    }
  }
  std::pair<uint64_t, uint64_t> timewindow(start_index_, end_index_);
  return timewindow;
}

inline uint64_t getLidarTimeWindow(const LidarContainer &lidar_container_,
                              const TimePoint T1) {
  uint64_t start_index_ = 0;
  for (size_t i = 0; i < lidar_container_.size(); i++) {
    if (lidar_container_[i].time_point == T1) {
      start_index_ = i;
      break;
    } else if (lidar_container_[i].time_point > T1) {
      start_index_ = i;
      break;
    } else if (i == lidar_container_.size()) {
      LOG_ERROR("getLidarTimeWindow: Cannot find lidar time window. Time "
                "inputs invalid.");
      return 0;
    }
  }
  return start_index_;
}

} // namespace wave

#endif // IG_GRAPH_SLAM_MEASUREMENTTYPES_HPP
