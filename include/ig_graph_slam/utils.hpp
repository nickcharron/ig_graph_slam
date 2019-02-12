#ifndef UTILS_HPP
#define UTILS_HPP

#include <chrono>
#include <cmath>
#include <ctime>
#include <math.h>
#include <wave/utils/math.hpp>

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

inline void outputTimePoint(const TimePoint t, const std::string output_text) {
  LOG_INFO("%s %f", output_text, t.time_since_epoch().count());
}

inline bool isRotationMatrix(Eigen::Matrix3d R) {
  Eigen::Matrix3d shouldBeIdentity = R * R.transpose();
  double detR = R.determinant();

  if (shouldBeIdentity.isIdentity() && detR == 1) {
    return 1;
  } else {
    return 0;
  }
}

inline double calculateLength(const Eigen::Vector3d &p1,
                              const Eigen::Vector3d &p2) {
  return sqrt((p1(0, 0) - p2(0, 0)) * (p1(0, 0) - p2(0, 0)) +
              (p1(1, 0) - p2(1, 0)) * (p1(1, 0) - p2(1, 0)) +
              (p1(2, 0) - p2(2, 0)) * (p1(2, 0) - p2(2, 0)));
}

inline bool isTransformationMatrix(Eigen::Matrix4d T) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  bool homoFormValid, tValid;

  // check translation for infinity or nan
  if (std::isinf(T(0, 3)) || std::isinf(T(1, 3)) || std::isinf(T(2, 3)) ||
      std::isnan(T(0, 3)) || std::isnan(T(1, 3)) || std::isnan(T(2, 3))) {
    tValid = 0;
  } else {
    tValid = 1;
  }

  // check that bottom row is [0 0 0 1]
  if (T(3, 0) == 0 && T(3, 1) == 0 && T(3, 2) == 0 && T(3, 3) == 1) {
    homoFormValid = 1;
  } else {
    homoFormValid = 0;
  }

  if (homoFormValid && tValid && isRotationMatrix(R)) {
    return 1;
  } else {
    return 0;
  }
}

inline void outputPercentComplete(int current_, int total_,
                                  std::string message_) {
  int out25, out50, out75;
  out25 = round(0.25 * total_);
  out50 = round(0.5 * total_);
  out75 = round(0.75 * total_);

  if (current_ == 1) {
    LOG_INFO("%s", message_.c_str());
    LOG_INFO("0 %% complete (1 of %d) ...", total_);
  } else if (current_ == out25) {
    LOG_INFO("25 %% complete (%d of %d) ...", current_, total_);
  } else if (current_ == out50) {
    LOG_INFO("50 %% complete (%d of %d) ...", current_, total_);
  } else if (current_ == out75) {
    LOG_INFO("75 %% complete (%d of %d) ...", current_, total_);
  } else if (current_ == total_) {
    LOG_INFO("100 %% complete (%d of %d)", current_, total_);
  }
}

inline wave::Mat4 interpolateTransform(const wave::Mat4 &m1,
                                       const TimePoint &t1,
                                       const wave::Mat4 &m2,
                                       const TimePoint &t2,
                                       const TimePoint &t) {
  double w2 = 1.0 * (t - t1) / (t2 - t1);

  wave::Mat4 T1 = m1;
  wave::Mat4 T2 = m2;
  wave::Mat4 T;

  wave::Mat3 R1 = T1.block<3, 3>(0, 0);
  wave::Mat3 R2 = T2.block<3, 3>(0, 0);
  wave::Mat3 R = (R2 * R1.transpose()).pow(w2) * R1;

  wave::Vec4 tr1 = T1.rightCols<1>();
  wave::Vec4 tr2 = T2.rightCols<1>();
  wave::Vec4 tr = (1 - w2) * tr1 + w2 * tr2;

  T.setIdentity();
  T.block<3, 3>(0, 0) = R;
  T.rightCols<1>() = tr;

  return T;
}

inline void outputTransform(Eigen::Affine3d T, std::string name) {
  std::cout << name << " :" << std::endl;
  std::cout << T.matrix() << std::endl;
}

#endif // UTILS_HPP
