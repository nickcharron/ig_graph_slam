#ifndef IG_GRAPH_SLAM_CONVERSIONS_HPP
#define IG_GRAPH_SLAM_CONVERSIONS_HPP

#include <chrono>
#include <ctime>
#include <cmath>
#include <wave/utils/math.hpp>
#include <wave_spatial_utils/world_frame_conversions.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#define DEG_TO_RAD 0.0174532925199433
#define RAD_TO_DEG 57.2957795130823209

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

const uint32_t WEEK_TO_SEC = 604800;
const uint32_t EPOCH_OFFSET = 315964800;
const uint32_t LEAP_SECONDS = 18;

inline void outputTimePoint(const TimePoint t, std::string output_text)
{
  std::cout << output_text << t.time_since_epoch().count() << std::endl;
}

inline void outputPercentComplete(int current_, int total_, std::string message_)
{
  int out25, out50, out75;
  out25 = round(0.25*total_);
  out50 = round(0.5*total_);
  out75 = round(0.75*total_);

    if(current_ == 1){LOG_INFO("%s", message_.c_str());}
    else if (current_ == out25){LOG_INFO("25 %% complete (%d of %d) ...", current_, total_);}
    else if (current_ == out50){LOG_INFO("50 %% complete (%d of %d) ...", current_, total_);}
    else if (current_ == out75){LOG_INFO("75 %% complete (%d of %d) ...", current_, total_);}
    else if (current_ == total_){LOG_INFO("100 %% complete (%d of %d)", current_, total_);}
}

inline wave::Mat4 interpolateTransform(const wave::Mat4 &m1, const TimePoint &t1,
                                       const wave::Mat4 &m2, const TimePoint &t2,
                                       const TimePoint &t)
{
  double w2 = 1.0 * (t - t1) / (t2 - t1);

  wave::Mat4 T1 = m1;
  wave::Mat4 T2 = m2;
  wave::Mat4 T;

  wave::Mat3 R1 = T1.block<3,3>(0,0);
  wave::Mat3 R2 = T2.block<3,3>(0,0);
  wave::Mat3 R = (R2 * R1.transpose()).pow(w2) * R1;

  wave::Vec4 tr1 = T1.rightCols<1>();
  wave::Vec4 tr2 = T2.rightCols<1>();
  wave::Vec4 tr = (1-w2) * tr1 + w2 * tr2;

  T.setIdentity();
  T.block<3,3>(0,0) = R;
  T.rightCols<1>()= tr;

  return T;
}

inline void outputTransform(Eigen::Affine3d T, std::string name)
{
  std::cout << name << " :" << std::endl;
  std::cout << T.matrix() << std::endl;
}

inline void ecefPointFromLLH(const double llh[3], double ecef[3])
{
    double latitude = llh[0], longitude = llh[1], height = llh[2];

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

    double X, Y, Z;
    earth.Forward(latitude*RAD_TO_DEG, longitude*RAD_TO_DEG, height, X, Y, Z);

    ecef[0] = X;
    ecef[1] = Y;
    ecef[2] = Z;
}

inline TimePoint rosTimeToChrono(const std_msgs::Header &hdr)
{
    std::chrono::seconds secs(hdr.stamp.sec);
    std::chrono::nanoseconds nsecs(hdr.stamp.nsec);
    auto dur = secs + nsecs;
    return TimePoint(dur);
}

inline Eigen::AngleAxisd enu_to_ecef_rotation(const double *lla)
{
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

inline TimePoint gpsTimeToChrono(const uint32_t gps_week, const uint32_t gps_milliseconds)
{
    uint64_t secs = WEEK_TO_SEC * gps_week + (gps_milliseconds / 1000) + EPOCH_OFFSET -
            LEAP_SECONDS;
    uint64_t nsecs = (gps_milliseconds % 1000) * 1000000;
    std::chrono::seconds Secs(secs);
    std::chrono::nanoseconds Nsecs(nsecs);
    auto dur = Secs + Nsecs;
    return TimePoint(dur);
}

#endif  // IG_GRAPH_SLAM_CONVERSIONS_HPP
