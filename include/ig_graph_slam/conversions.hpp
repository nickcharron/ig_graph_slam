#ifndef IG_GRAPH_SLAM_CONVERSIONS_HPP
#define IG_GRAPH_SLAM_CONVERSIONS_HPP

// #include <iostream>
// #include <chrono>
// #include <ctime>
#include <wave/containers/measurement_container.hpp>
#include <wave/utils/math.hpp>
#include <wave_spatial_utils/world_frame_conversions.hpp>
#include <ros/ros.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#define DEG_TO_RAD 0.0174532925199433
#define RAD_TO_DEG 57.2957795130823209

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

const uint32_t WEEK_TO_SEC = 604800;
const uint32_t EPOCH_OFFSET = 315964800;
const uint32_t LEAP_SECONDS = 18;

//void outputTimePoint(TimePoint time_point, std::string caption)
// void outputTimePoint(TimePoint time_point)
// {
//   std::time_t tp = std::chrono::time_point<Clock>::to_time_t(time_point);
//   std::cout << "Time: " << std::ctime(&tp);
// }

void outputTransform(Eigen::Affine3d T, std::string name)
{
  std::cout << name << " :" << std::endl;
  std::cout << T.matrix() << std::endl;
}

void ecefPointFromLLH(const double llh[3], double ecef[3])
{
    double latitude = llh[0], longitude = llh[1], height = llh[2];

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

    double X, Y, Z;
    earth.Forward(latitude*RAD_TO_DEG, longitude*RAD_TO_DEG, height, X, Y, Z);

    ecef[0] = X;
    ecef[1] = Y;
    ecef[2] = Z;
}

TimePoint rosTimeToChrono(const std_msgs::Header &hdr)
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

inline Eigen::Affine3d gpsToEigen(const Eigen::Matrix<double, 6, 1> measurement,
                                  bool applyT_ENU_GPS)
{
    double T_ECEF_ENU[4][4];
    wave_spatial_utils::ecefFromENUTransformMatrix(measurement.data(), T_ECEF_ENU, true);

    // Transformation from the ENU frame formed at the position of the GPS
    // antenna to ECEF
    Eigen::Affine3d E_T_ECEF_ENU;
    E_T_ECEF_ENU.matrix() << T_ECEF_ENU[0][0],
            T_ECEF_ENU[0][1], T_ECEF_ENU[0][2], T_ECEF_ENU[0][3],
            T_ECEF_ENU[1][0], T_ECEF_ENU[1][1], T_ECEF_ENU[1][2],
            T_ECEF_ENU[1][3], T_ECEF_ENU[2][0], T_ECEF_ENU[2][1],
            T_ECEF_ENU[2][2], T_ECEF_ENU[2][3], T_ECEF_ENU[3][0],
            T_ECEF_ENU[3][1], T_ECEF_ENU[3][2], T_ECEF_ENU[3][3];

    //outputTransform(E_T_ECEF_ENU, "E_T_ECEF_ENU");

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

    //outputTransform(T_ENU_GPSIMU, "T_ENU_GPSIMU");
    //outputTransform(E_T_ECEF_ENU * T_ENU_GPSIMU, "T_ECEF_GPSIMU");
    Eigen::Affine3d T_ECEF_GPSIMU;
    if (applyT_ENU_GPS)
    {
        return E_T_ECEF_ENU * T_ENU_GPSIMU;
    }
    else
    {
        return E_T_ECEF_ENU;
    }

}

// this is only used for INSPVAX messages
TimePoint gpsTimeToChrono(const uint32_t gps_week, const uint32_t gps_milliseconds)
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
