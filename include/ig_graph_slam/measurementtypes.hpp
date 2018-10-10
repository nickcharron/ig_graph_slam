#ifndef IG_GRAPH_SLAM_MEASUREMENTTYPES_HPP
#define IG_GRAPH_SLAM_MEASUREMENTTYPES_HPP

#include <utility>
#include <chrono>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pointmatcher/PointMatcher.h>

#include <wave/utils/math.hpp>
#include <wave/containers/measurement.hpp>

using GPSMeasurement = wave::Measurement<std::pair<wave::Vec6, wave::Vec6>, uint>;
using PCLLidarMeasurement = wave::Measurement<wave::PCLPointCloudPtr, uint>;
//using LPMLidarMeasurement = wave::Measurement<boost::shared_ptr<PointMatcher<double>::DataPoints>, uint>;
using PoseMeasurement = wave::Measurement<Eigen::Affine3d, uint>;
using TimeType = std::chrono::steady_clock::time_point;

namespace wave
{

  double wrapToPI(double theta)
  {
      double retval = theta;
      if (theta > M_PI)
      {
          retval -= 2 * M_PI;
      }
      else if (theta < -M_PI)
      {
          retval += 2*M_PI;
      }
      return retval;
  }

  decltype(GPSMeasurement::value) interpolate(const GPSMeasurement &m1,
                                              const GPSMeasurement &m2,
                                              const TimeType &t)
  {
      double w2 = 1.0 * (t - m1.time_point) / (m2.time_point - m1.time_point);

      wave::Vec6 first = (1 - w2) * std::get<0>(m1.value) + w2 * std::get<0>(m2.value);
      wave::Vec6 second = (1 - w2) * std::get<1>(m1.value) + w2 * std::get<1>(m2.value);

      auto yaw1 = std::get<0>(m1.value)(5);
      auto yaw2 = std::get<0>(m2.value)(5);

      double delta = yaw2 - yaw1;
      delta = wrapToPI(delta);

      first(5) = yaw1 + w2*delta;
      first(5) = wrapToPI(first(5));

      return std::make_pair(first, second);
  }

}  // namespace wave

#endif //IG_GRAPH_SLAM_MEASUREMENTTYPES_HPP
