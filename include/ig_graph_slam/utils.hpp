#ifndef UTILS_HPP
#define UTILS_HPP

#include "conversions.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <math.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <utility>

// libbeam specific headers
#include <beam/utils/log.hpp>
#include <beam/utils/math.hpp>

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

inline void outputTimePoint(const TimePoint t, const std::string output_text) {
  LOG_INFO("%s %f", output_text, t.time_since_epoch().count());
}

inline void
outputTimePointDiff(const std::chrono::system_clock::time_point tStart,
                    const std::chrono::system_clock::time_point tEnd,
                    const std::string output_text) {
  double time_diff =
      (tEnd.time_since_epoch().count() - tStart.time_since_epoch().count()) /
      1000000000;
  int time_diff_mins = std::floor(time_diff / 60);
  int time_diff_secs = std::round((time_diff / 60 - time_diff_mins) * 60);
  LOG_INFO("%s %dm:%ds", output_text.c_str(), time_diff_mins, time_diff_secs);
}

inline double calculateLength(const Eigen::Vector3d &p1,
                              const Eigen::Vector3d &p2) {
  return sqrt((p1(0, 0) - p2(0, 0)) * (p1(0, 0) - p2(0, 0)) +
              (p1(1, 0) - p2(1, 0)) * (p1(1, 0) - p2(1, 0)) +
              (p1(2, 0) - p2(2, 0)) * (p1(2, 0) - p2(2, 0)));
}

inline double calculateLength(const Eigen::Affine3d &p1,
                              const Eigen::Affine3d &p2) {
  return sqrt((p1.matrix()(0, 3) - p2.matrix()(0, 3)) *
                  (p1.matrix()(0, 3) - p2.matrix()(0, 3)) +
              (p1.matrix()(1, 3) - p2.matrix()(1, 3)) *
                  (p1.matrix()(1, 3) - p2.matrix()(1, 3)) +
              (p1.matrix()(2, 3) - p2.matrix()(2, 3)) *
                  (p1.matrix()(2, 3) - p2.matrix()(2, 3)));
}

inline std::vector<double>
calculateMinMaxRotationChange(const Eigen::Affine3d &p1,
                              const Eigen::Affine3d &p2) {
  Eigen::Vector3d eps1, eps2, diffSq;
  std::vector<double> minmax;
  double minsq, maxsq;
  eps1 = RToLieAlgebra(p1.rotation());
  eps2 = RToLieAlgebra(p2.rotation());
  diffSq(0, 0) = (eps2(0, 0) - eps1(0, 0)) * (eps2(0, 0) - eps1(0, 0));
  diffSq(1, 0) = (eps2(1, 0) - eps1(1, 0)) * (eps2(1, 0) - eps1(1, 0));
  diffSq(2, 0) = (eps2(2, 0) - eps1(2, 0)) * (eps2(2, 0) - eps1(2, 0));
  minsq = diffSq.minCoeff();
  maxsq = diffSq.maxCoeff();
  minmax.push_back(sqrt(minsq));
  minmax.push_back(sqrt(maxsq));
  return minmax;
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

inline beam::Mat4 interpolateTransform(const beam::Mat4 &m1,
                                       const TimePoint &t1,
                                       const beam::Mat4 &m2,
                                       const TimePoint &t2,
                                       const TimePoint &t) {
  double w2 = 1.0 * (t - t1) / (t2 - t1);

  beam::Mat4 T1 = m1;
  beam::Mat4 T2 = m2;
  beam::Mat4 T;

  beam::Mat3 R1 = T1.block<3, 3>(0, 0);
  beam::Mat3 R2 = T2.block<3, 3>(0, 0);
  beam::Mat3 R = (R2 * R1.transpose()).pow(w2) * R1;

  beam::Vec4 tr1 = T1.rightCols<1>();
  beam::Vec4 tr2 = T2.rightCols<1>();
  beam::Vec4 tr = (1 - w2) * tr1 + w2 * tr2;

  T.setIdentity();
  T.block<3, 3>(0, 0) = R;
  T.rightCols<1>() = tr;

  return T;
}

inline void outputTransform(Eigen::Affine3d T, std::string name) {
  std::cout << name << " :" << std::endl;
  std::cout << T.matrix() << std::endl;
}

inline std::vector<std::pair<uint64_t, Eigen::Matrix4d>>
readPoseFile(const std::string filename) {
  // declare variables
  std::ifstream infile;
  std::string line;
  Eigen::Matrix4d Tk;
  uint64_t tk;
  std::vector<std::pair<uint64_t, Eigen::Matrix4d>> poses;

  // open file
  infile.open(filename);

  // extract poses
  while (!infile.eof()) {
    // get timestamp k
    std::getline(infile, line, ',');
    try {
      tk = std::stod(line);
    } catch (const std::invalid_argument &e) {
      LOG_INFO("Invalid argument, probably at end of file");
      return poses;
    }

    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        if (i == 3 && j == 3) {
          std::getline(infile, line, '\n');
          Tk(i, j) = std::stod(line);
        } else {
          std::getline(infile, line, ',');
          Tk(i, j) = std::stod(line);
        }
      }
    }
    poses.push_back(std::make_pair(tk, Tk));
  }
  return poses;
}

#endif // UTILS_HPP
