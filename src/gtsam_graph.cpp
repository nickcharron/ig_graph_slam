#include "gtsam_graph.hpp"
#include <fstream>
#include <wave/gtsam/decaying_bias.hpp>
#include <wave/gtsam/hand_eye.hpp>

// libbeam specific headers
#include <beam/utils/math.hpp>

GTSAMGraph::GTSAMGraph() {}

void GTSAMGraph::clear() {
  this->graph.erase(this->graph.begin(), this->graph.end());
  this->initial.clear();
  this->poses.clear();
  this->biases.clear();
}

void GTSAMGraph::addFactor(uint64_t from, uint64_t to,
                           const Eigen::Affine3d &transform,
                           const beam::Mat6 &info) {
  gtsam::Key id1 = from;
  gtsam::Key id2 = to;
  gtsam::Pose3 pose(transform.matrix());
  auto noise_model = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(2.3849),
      gtsam::noiseModel::Gaussian::Information(info));
  //    auto noise_model = gtsam::noiseModel::Gaussian::Information(info);
  this->graph.add(
      gtsam::BetweenFactor<gtsam::Pose3>(id1, id2, pose, noise_model));
}

void GTSAMGraph::addInitialPose(const Eigen::Affine3d &pose, gtsam::Key id) {
  gtsam::Key key = id;
  gtsam::Pose3 init(pose.matrix());
  this->initial.insert(key, init);
  this->poses.emplace_back(key);
}

void GTSAMGraph::fixFirstPose() {
  gtsam::Vector6 v6;
  v6 << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
  gtsam::noiseModel::Diagonal::shared_ptr priorModel =
      gtsam::noiseModel::Diagonal::Variances(v6);
  gtsam::Key firstKey = 0;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      firstKey, this->initial.at(firstKey).cast<gtsam::Pose3>(), priorModel));
}

void GTSAMGraph::print(std::string fileName, bool printToTerminal) {
  if (printToTerminal) {
    graph.print();
  }
  std::ofstream graphfile(fileName);
  this->graph.saveGraph(graphfile);
  graphfile.close();
}

void GTSAMGraph::optimize() {
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("TERMINATION");
  params.absoluteErrorTol = 1e-8;
  params.relativeErrorTol = 1e-8;
  params.setlambdaUpperBound(1e8);
  gtsam::LevenbergMarquardtOptimizer optimizer(this->graph, this->initial,
                                               params);
  this->result.clear();
  std::exception_ptr eptr;
  try {
    this->result = optimizer.optimize();
  } catch (...) {
    this->graph.print();
    this->initial.print();

    std::ofstream graphfile("file.dot");
    this->graph.saveGraph(graphfile);
    graphfile.close();

    eptr = std::current_exception();
    std::rethrow_exception(eptr);
  }
}
