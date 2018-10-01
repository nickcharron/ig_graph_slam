#ifndef IG_GRAPH_SLAM_GTSAM_GRAPH_HPP
#define IG_GRAPH_SLAM_GTSAM_GRAPH_HPP

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <wave/utils/math.hpp>


class GTSAMGraph {
 public:
    GTSAMGraph();
};

#endif  // IG_GRAPH_SLAM_GTSAM_GRAPH_HPP
