#ifndef IG_GRAPH_SLAM_GTSAM_GRAPH_HPP
#define IG_GRAPH_SLAM_GTSAM_GRAPH_HPP

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <wave/utils/math.hpp>

class GTSAMGraph {
public:
  GTSAMGraph();

  // Declare Functions
  void clear();

  // add a factor between to poses
  void addFactor(uint64_t from, uint64_t to, const Eigen::Affine3d &transform,
                 const wave::Mat6 &info);

  void addInitialPose(const Eigen::Affine3d &pose, gtsam::Key id);
  void fixFirstPose();
  void optimize();
  void print(std::string fileName, bool printToTerminal);

  // Declare variables
  gtsam::Values initial, result;
  // gtsam::Key T_GPS_LIDAR_key;
  gtsam::NonlinearFactorGraph graph;
  std::vector<gtsam::Key> poses;
  std::vector<gtsam::Key> biases;
};

#endif // IG_GRAPH_SLAM_GTSAM_GRAPH_HPP
