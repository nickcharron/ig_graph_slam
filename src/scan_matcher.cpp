// ROS and other Headers
#include <boost/filesystem.hpp>
#include <fstream>
#include <math.h>
#include <sstream>
#include <string>
#include <unistd.h>

#include <chrono>
#include <ctime>

// PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// WAVE Headers
#include "wave/matching/gicp.hpp"
#include <wave/containers/measurement.hpp>
#include <wave/containers/measurement_container.hpp>
#include <wave/matching/icp.hpp>
#include <wave/matching/pointcloud_display.hpp>
#include <wave/utils/log.hpp>

// IG Graph SLAM Headers
#include "conversions.hpp"
#include "gtsam_graph.hpp"
#include "load_ros_data.hpp"
#include "measurementtypes.hpp"
#include "pcl_filters.hpp"
#include "scan_matcher.hpp"
#include "slam_params.hpp"
#include "utils.hpp"

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

// Scan Matcher (parent Class) Functions:

GTSAMGraph ScanMatcher::buildGTSAMGraph(boost::shared_ptr<ROSBag> ros_data) {
  // init. variables:
  GTSAMGraph graph;
  bool no_GPS = true;
  int cnt_match = 0;
  TimePoint last_timestamp;
  bool correction_norm_valid;

  // build graph
  for (int outer_loops = 0; outer_loops < this->params.iterations;
       outer_loops++) {
    // Iterate to update initial estimates and redo matches
    LOG_INFO("Iteration No. %d of %d.", outer_loops + 1,
             this->params.iterations);
    cnt_match = 0;
    graph.clear();
    graph.result.clear();

    // Determine how many scans to register against for each pose and save
    findAdjacentScans();

    // Determine how many scans to register against for each pose and save
    findLoops();

    // iterate over all j scans
    for (uint64_t j = 0; j < this->adjacency->size(); j++) {
      if (j == 0) {
        last_timestamp =
            ros_data->getLidarScanTimePoint(this->pose_scan_map.at(j));
      }

      // iterate over all scans adjacent to scan j
      for (uint64_t iter = 0; iter < this->adjacency->at(j).size(); iter++) {
        Eigen::Affine3d T_Liter_Lj;
        wave::Mat6 info;
        bool match_success = false;

        // Attempts to match the scans and checks the correction norm
        // between scan transformation and GPS
        match_success =
            this->matchScans(this->adjacency->at(j).at(iter), j, T_Liter_Lj,
                             info, correction_norm_valid, ros_data);

        if (!correction_norm_valid) // WHAT IS THIS??
        {
          continue;
        }
        if (match_success) // create factor in graph if scan successful
        {
          wave::Mat6 mgtsam; // Eigen::Matrix<double, 6, 6>
          // this next bit is a major gotcha, whole thing blows up
          // without it. This just rearranges the info matrix to the correct
          // form block starting at 0,0 of size 3x3
          mgtsam.block(0, 0, 3, 3) = info.block(3, 3, 3, 3); // rotation
          mgtsam.block(3, 3, 3, 3) = info.block(0, 0, 3, 3); // translation
          mgtsam.block(0, 3, 3, 3) = info.block(0, 3, 3, 3); // off diagonal
          mgtsam.block(3, 0, 3, 3) = info.block(3, 0, 3, 3); // off diagonal
          graph.addFactor(this->adjacency->at(j).at(iter), j, T_Liter_Lj,
                          mgtsam);
          outputPercentComplete(cnt_match, this->total_matches,
                                "Matching scans between nearest neighbours...");
          cnt_match++;
        } else {
          LOG_ERROR("Scan match failed");
        }
      }

      if (this->params.use_gps) {
        // only if we want to use gps priors
        // TODO: Implement the gps priors.
        // Careful because currently GPS has no rotation info
        // See Ben's code
      }
      graph.addInitialPose(this->init_pose.at(j).value, j);
    }

    for (uint64_t j = 0; j < this->loops->size(); j++) {
      // iterate over all j loop closures
      Eigen::Affine3d T_L1_L2;
      wave::Mat6 info;
      bool match_success;

      uint64_t L1 = this->loops->at(j)[0];
      uint64_t L2 = this->loops->at(j)[1];

      match_success = this->matchScans(L1, L2, T_L1_L2, info,
                                       correction_norm_valid, ros_data);

      if (!correction_norm_valid) // WHAT IS THIS??
      {
        continue;
      }

      // create factor in graph if scan successful
      if (match_success) {
        wave::Mat6 mgtsam; // Eigen::Matrix<double, 6, 6>
        // this next bit is a major gotcha, whole thing blows up
        // without it. This just rearranges the info matrix to the correct form
        // block starting at 0,0 of size 3x3
        mgtsam.block(0, 0, 3, 3) = info.block(3, 3, 3, 3); // rotation
        mgtsam.block(3, 3, 3, 3) = info.block(0, 0, 3, 3); // translation
        mgtsam.block(0, 3, 3, 3) = info.block(0, 3, 3, 3); // off diagonal
        mgtsam.block(3, 0, 3, 3) = info.block(3, 0, 3, 3); // off diagonal
        graph.addFactor(L1, L2, T_L1_L2, mgtsam);
        outputPercentComplete(cnt_match, this->total_matches,
                              "Matching scans between nearest neighbours...");
        cnt_match++;
      } else {
        LOG_ERROR("Scan match failed");
      }
    }

    if (no_GPS) {
      LOG_INFO("Fixing first pose.");
      graph.fixFirstPose();
    }
    LOG_INFO("Done building graph.");
    graph.optimize();

    // Loop through and get final alignment
    LOG_INFO("Updating Poses for Next Iteration.");
    this->final_poses.clear();
    Eigen::Affine3d temp_trans, prev;
    for (uint64_t k = 0; k < graph.poses.size(); k++) {
      // get resulting transform for pose k
      temp_trans.matrix() =
          graph.result.at<gtsam::Pose3>(graph.poses.at(k)).matrix();

      // update initial pose estimate for next iteration (if needed)
      this->init_pose.at(graph.poses.at(k)).value = temp_trans;

      // Add result to final pose measurement container
      this->final_poses.emplace_back(
          ros_data->getLidarScanTimePoint(this->pose_scan_map[k]), 0,
          temp_trans);
    }
  }

  return graph;
}

void ScanMatcher::saveParamsFile(std::string save_path_) {
  std::string dateandtime = convertTimeToDate(std::chrono::system_clock::now());
  std::string dstFileName = save_path_ + dateandtime + "_params.txt";
  std::string yamlDirStr = __FILE__;
  yamlDirStr.erase(yamlDirStr.end() - 20, yamlDirStr.end());
  yamlDirStr += "config/ig_graph_slam_config.yaml";
  std::ifstream src(yamlDirStr, std::ios::binary);
  std::ofstream dst(dstFileName, std::ios::binary);
  dst << src.rdbuf();
}

void ScanMatcher::saveGraphFile(GTSAMGraph graph_, std::string save_path_) {
  std::string dateandtime = convertTimeToDate(std::chrono::system_clock::now());
  std::string graphFileName = save_path_ + dateandtime + "gtsam_graph.dot";
  graph_.print(graphFileName, false);
}

bool ScanMatcher::takeNewScan(const Eigen::Affine3d &p1,
                              const Eigen::Affine3d &p2, const double &dist,
                              const double &rot) {
  // calculate the norm of the distance between the two points
  double l2sqrd = (p1(0, 3) - p2(0, 3)) * (p1(0, 3) - p2(0, 3)) +
                  (p1(1, 3) - p2(1, 3)) * (p1(1, 3) - p2(1, 3)) +
                  (p1(2, 3) - p2(2, 3)) * (p1(2, 3) - p2(2, 3));

  double minRotSq = rot * DEG_TO_RAD * rot * DEG_TO_RAD;
  Eigen::Vector3d eps1, eps2, diffSq;
  eps1 = RToLieAlgebra(p1.rotation());
  eps2 = RToLieAlgebra(p2.rotation());
  diffSq(0, 0) = (eps2(0, 0) - eps1(0, 0)) * (eps2(0, 0) - eps1(0, 0));
  diffSq(1, 0) = (eps2(1, 0) - eps1(1, 0)) * (eps2(1, 0) - eps1(1, 0));
  diffSq(2, 0) = (eps2(2, 0) - eps1(2, 0)) * (eps2(2, 0) - eps1(2, 0));

  // if the norm is greater than the specified minimum sampling distance or
  // if the change in rotation is greater than specified min.
  if (l2sqrd > dist * dist) {
    // then yes take a new scan
    return true;
  } else if (diffSq(0, 0) > minRotSq || diffSq(1, 0) > minRotSq ||
             diffSq(2, 0) > minRotSq) {
    // then yes take a new scan
    return true;
  } else {
    // then no, do not take a new scan
    return false;
  }
}

void ScanMatcher::loadPrevPoses() {
  Eigen::Affine3d PoseK;
  uint64_t tk;
  TimePoint timepointk;
  std::vector<std::pair<uint64_t, Eigen::Matrix4d>> poses_;

  poses_ = readPoseFile(this->params.prev_poses_path);
  LOG_INFO("Loaded a total of %d poses.", poses_.size());

  for (uint64_t k = 0; k < poses_.size(); k++) {
    // get resulting transform and timestamp for pose k
    tk = poses_[k].first;
    PoseK.matrix() = poses_[k].second;
    TimePoint timepointk{std::chrono::duration_cast<TimePoint::duration>(
        std::chrono::nanoseconds(tk))};

    // Add result to final pose measurement container
    this->final_poses.emplace_back(timepointk, 0, PoseK);
  }
}

void ScanMatcher::createPoseScanMap(boost::shared_ptr<ROSBag> ros_data) {
  LOG_INFO("Storing pose scans...");
  // save initial poses of the lidar scans based on GPS data and save
  // iterators corresponding to these scans
  int i = 0;
  Eigen::Affine3d T_ECEF_GPS, T_MAP_LIDAR;

  // this ierates through the lidar measurements
  for (uint64_t iter = 0; iter < ros_data->lidar_container.size(); iter++) {
    bool use_next_scan = false;
    switch (this->params.init_method) {
    case 1:
      try {
        // extract gps measurement at the same timepoint as the current
        // lidar message
        auto gps_pose = ros_data->gps_container.get(
            ros_data->lidar_container[iter].time_point, 0);
        T_ECEF_GPS = gpsToEigen(gps_pose.first, true); // true: apply T_ENU_GPS
        T_MAP_LIDAR = ros_data->T_ECEF_MAP.inverse() * T_ECEF_GPS *
                      this->params.T_LIDAR_GPS.inverse();
      } catch (const std::out_of_range &e) {
        LOG_INFO("No gps pose for time of scan, may happen at edges of "
                 "recorded data");
        use_next_scan = true;
        break;
      }
      break;
    case 2:
      try {
        // extract odometry pose at the same timepoint as current lidar
        // message
        auto odom_pose = ros_data->odom_container.get(
            ros_data->lidar_container[iter].time_point, 2);
        T_MAP_LIDAR = odom_pose.first;
      } catch (const std::out_of_range &e) {
        LOG_INFO("No odometry message for time of scan, may happen at edges of "
                 "recorded data");
        use_next_scan = true;
        break;
      }
      break;
    }

    // If i > 0 then check to see if the distance between current scan and
    // last scan is greater than the minimum, if so then save this pose
    // If i = 0, then save the scan - first scan
    if (i > 0) {
      bool take_new_scan;
      take_new_scan = takeNewScan(T_MAP_LIDAR, init_pose.at(i - 1).value,
                                  this->params.trajectory_sampling_dist,
                                  this->params.trajectory_rotation_change);
      if (take_new_scan && !use_next_scan) {
        this->init_pose.emplace_back(ros_data->getLidarScanTimePoint(iter), 0,
                                     T_MAP_LIDAR);
        this->pose_scan_map.push_back(iter);
        ++i;
      }
    } else if (!use_next_scan) {
      this->init_pose.emplace_back(ros_data->getLidarScanTimePoint(iter), 0,
                                   T_MAP_LIDAR);
      this->pose_scan_map.push_back(iter);
      ++i;
    }
  }
  LOG_INFO("Stored %d pose scans of %d available scans.", i,
           ros_data->lidar_container.size());
}

void ScanMatcher::createPoseScanMapFromPoses(
    boost::shared_ptr<ROSBag> ros_data) {
  LOG_INFO("Creating pose scan map from inputted poses...");
  uint64_t index;
  TimePoint timepointk;
  for (uint64_t i = 0; i < this->final_poses.size(); i++) {
    timepointk = this->final_poses[i].time_point;
    index = getLidarTimeWindow(ros_data->lidar_container, timepointk);
    this->pose_scan_map.push_back(index);
  }
  LOG_INFO("Stored %d pose scans of %d available scans.",
           this->pose_scan_map.size(), ros_data->lidar_container.size());
}

void ScanMatcher::findAdjacentScans() {
  double distancejk;
  std::vector<double> minmaxrotjk;
  Eigen::Affine3d posej, posek;
  this->total_matches = 0; // reset the counter of matches

  // Scan registration class
  // As factors are added, factor id will just be counter value
  this->adjacency = boost::make_shared<std::vector<std::vector<uint64_t>>>(
      this->init_pose.size());

  /*
   * For each pose within the initial pose, get the position.
   * Connect all the positions until the last pose by adding it to the
   * adjacency at that point
   *
   */
  for (uint64_t j = 0; j < this->init_pose.size(); j++) {
    // posej is the position of the current pose
    posej = this->init_pose.at(j).value;
    // Do this for all poses except the last one
    if (j + 1 < this->init_pose.size()) {
      // ensures that trajectory is connected
      //  this is also accounted for in nn
      //  search (see if j < ret_indices[k]-1)
      this->adjacency->at(j).emplace_back(j + 1);
      this->total_matches++;
    }

    /*
     * For each pose position, check the next scans to see if they are
     * within the boundaries specified
     * If they are and they are after j, add them to the adjacency
     *
     */
    for (uint16_t k = j + 2;
         (k < j + 1 + this->params.knn) && (k < this->init_pose.size()); k++) {
      posek = this->init_pose.at(k).value;
      distancejk = calculateLength(posej, posek);
      minmaxrotjk = calculateMinMaxRotationChange(posej, posek);

      if ((distancejk > this->params.distance_match_min) &&
          (distancejk < this->params.distance_match_limit &&
           minmaxrotjk[1] < this->params.rotation_match_limit * DEG_TO_RAD)) {
        // add index to back of vector for scan j
        this->adjacency->at(j).emplace_back(k);
        this->total_matches++;
      } else if (minmaxrotjk[1] >
                     this->params.rotation_match_min * DEG_TO_RAD &&
                 minmaxrotjk[1] <
                     this->params.rotation_match_limit * DEG_TO_RAD &&
                 distancejk < this->params.distance_match_limit) {
        // add index to back of vector for scan j
        this->adjacency->at(j).emplace_back(k);
        this->total_matches++;
      }
    }
  }
}

void ScanMatcher::findLoops() {
  int knn_ = this->params.knn;
  double pathLength, distanceP1P2;
  Eigen::Vector3d pose1, pose2, poseLast;
  std::vector<uint64_t> loopIndices = {0, 0};

  // creating a vector (size nx1) of vectors (will be size 2x1)
  this->loops = boost::make_shared<std::vector<std::vector<uint64_t>>>();

  for (uint64_t j = 0; j < this->init_pose.size() - knn_; j++) {
    pathLength = 0;
    distanceP1P2 = 0;
    pose1(0, 0) = this->init_pose.at(j).value(0, 3);
    pose1(1, 0) = this->init_pose.at(j).value(1, 3);
    pose1(2, 0) = this->init_pose.at(j).value(2, 3);
    poseLast = pose1;

    // For each pose (pose1) position, check all the poses j + 1 and up
    // if within loop_max_distance and outside of loop_min_travel_distance,
    // then add the constraint (or add to loops object)
    for (uint16_t k = j + 1; k < init_pose.size(); k++) {
      pose2(0, 0) = this->init_pose.at(k).value(0, 3);
      pose2(1, 0) = this->init_pose.at(k).value(1, 3);
      pose2(2, 0) = this->init_pose.at(k).value(2, 3);

      distanceP1P2 = calculateLength(pose1, pose2);
      pathLength += calculateLength(poseLast, pose2);
      poseLast = pose2;

      if (distanceP1P2 < this->params.loop_max_distance &&
          pathLength > this->params.loop_min_travel_distance) {
        loopIndices = {j, k};
        this->loops->emplace_back(loopIndices);
        this->total_matches++;
      }
    }
  }
  LOG_INFO("Found a total of %d loop closure scans.", this->loops->size());
}

void ScanMatcher::displayPointCloud(wave::PCLPointCloudPtr cloud_display,
                                    int color,
                                    const Eigen::Affine3d &transform) {
  if (this->params.visualize) {
    *this->cloud_temp_display = *cloud_display;
    if (!transform.matrix().isIdentity()) {
      pcl::transformPointCloud(*cloud_display, *this->cloud_temp_display,
                               transform);
    }
    this->init_display.addPointcloud(this->cloud_temp_display, color);
  }
}

void ScanMatcher::createAggregateMap(boost::shared_ptr<ROSBag> ros_data,
                                     int mapping_method) {
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> intermediaries;
  this->aggregate->clear();
  int i = 0;
  for (uint64_t k = 0; k < this->final_poses.size(); k++) {
    // Define transforms we will need
    Eigen::Affine3d T_MAP_LLOC_k, T_MAP_LLOC_kp1, T_MAP_LMAP_k, T_MAP_LMAP_kp1,
        T_LMAP_LLOC, T_MAP_LLOC_Jprev, T_MAP_LMAP_Jprev;

    T_MAP_LLOC_k = this->final_poses[k].value;
    T_LMAP_LLOC = this->params.T_LMAP_LLOC; // static
    T_MAP_LMAP_k = T_MAP_LLOC_k * T_LMAP_LLOC.inverse();

    int curr_index = this->pose_scan_map.at(k);
    TimePoint curr_pose_time = ros_data->lidar_container[curr_index].time_point;
    int next_index;
    TimePoint next_pose_time = curr_pose_time;

    // get all time and transforms for next pose for interpolation
    if (!(k == this->final_poses.size() - 1)) {
      T_MAP_LLOC_kp1 = this->final_poses[k + 1].value; // for pose k + 1
      T_MAP_LMAP_kp1 = T_MAP_LLOC_kp1 * T_LMAP_LLOC.inverse();
      T_MAP_LLOC_Jprev = T_MAP_LLOC_k;
      T_MAP_LMAP_Jprev = T_MAP_LMAP_k;
      next_index = this->pose_scan_map.at(k + 1);
      next_pose_time = ros_data->lidar_container[next_index].time_point;
    }

    // find scan range for map scans:
    std::pair<uint64_t, uint64_t> scan_range_map;
    scan_range_map = getLidarTimeWindow(ros_data->lidar_container_map,
                                        curr_pose_time, next_pose_time);

    switch (mapping_method) {
    case 1:
      // transform current pose scan to target cloud
      pcl::transformPointCloud(*(ros_data->lidar_container[curr_index].value),
                               *this->cloud_target, T_MAP_LLOC_k);

      // iterate through all scans between pose k and k+1
      if ((this->params.trajectory_sampling_dist >
           this->params.map_sampling_dist) &&
          !(k == this->final_poses.size() - 1)) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_interp(
            new pcl::PointCloud<pcl::PointXYZ>);

        int j = 0;
        while (true) {
          j++;
          Eigen::Affine3d T_MAP_LLOC_J; // interpolated scan pose
          TimePoint time_point_J =
              ros_data->lidar_container[curr_index + j].time_point;

          if (time_point_J >= next_pose_time) // stop interpolation
          {
            break;
          }

          T_MAP_LLOC_J.matrix() = interpolateTransform(
              T_MAP_LLOC_k.matrix(), curr_pose_time, T_MAP_LLOC_kp1.matrix(),
              next_pose_time, time_point_J);

          bool take_new_map_scan = takeNewScan(
              T_MAP_LLOC_Jprev, T_MAP_LLOC_J, this->params.map_sampling_dist,
              this->params.map_rotation_change);

          // interpolate pose and add new scan to current target cloud
          if (take_new_map_scan) {
            pcl::transformPointCloud(
                *(ros_data->lidar_container[curr_index + j].value),
                *cloud_interp, T_MAP_LLOC_J);
            *cloud_target += *cloud_interp;
            T_MAP_LLOC_Jprev = T_MAP_LLOC_J;
          }
        }
      }
      break;
    case 2:
      // transform current pose scan to target cloud
      pcl::transformPointCloud(
          *(ros_data->lidar_container_map[scan_range_map.first].value),
          *this->cloud_target, T_MAP_LMAP_k);

      // iterate through all scans between pose k and k+1
      if ((this->params.trajectory_sampling_dist >
           this->params.map_sampling_dist) &&
          !(k == this->final_poses.size() - 1)) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_interp(
            new pcl::PointCloud<pcl::PointXYZ>);

        int j = 0;
        while (true) {
          j++;
          Eigen::Affine3d T_MAP_LMAP_J; // interpolated scan pose
          TimePoint time_point_J =
              ros_data->lidar_container_map[scan_range_map.first + j]
                  .time_point;

          if (time_point_J >= next_pose_time) // stop interpolation
          {
            break;
          }

          T_MAP_LMAP_J.matrix() = interpolateTransform(
              T_MAP_LMAP_k.matrix(), curr_pose_time, T_MAP_LMAP_kp1.matrix(),
              next_pose_time, time_point_J);
          bool take_new_map_scan = takeNewScan(
              T_MAP_LMAP_Jprev, T_MAP_LMAP_J, this->params.map_sampling_dist,
              this->params.map_rotation_change);

          // interpolate pose and add new scan to current target cloud
          if (take_new_map_scan) {
            pcl::transformPointCloud(
                *(ros_data->lidar_container_map[scan_range_map.first + j]
                      .value),
                *cloud_interp, T_MAP_LMAP_J);
            *cloud_target += *cloud_interp;
            T_MAP_LMAP_Jprev = T_MAP_LMAP_J;
          }
        }
      }
      break;
    case 3:

      // transform current map scan to tmp cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(
          *(ros_data->lidar_container_map[scan_range_map.first].value),
          *cloud_tmp, T_MAP_LMAP_k);
      // transform current localization scan to target cloud
      pcl::transformPointCloud(*(ros_data->lidar_container[curr_index].value),
                               *this->cloud_target, T_MAP_LLOC_k);
      // Add tmp cloud to target cloud
      *this->cloud_target += *cloud_tmp;

      // iterate through all scans between pose k and k+1
      if ((this->params.trajectory_sampling_dist >
           this->params.map_sampling_dist) &&
          !(k == this->final_poses.size() - 1)) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_interp(
            new pcl::PointCloud<pcl::PointXYZ>);
        int j = 0;
        while (true) {
          j++;
          Eigen::Affine3d T_MAP_LLOC_J,
              T_MAP_LMAP_J; // interpolated scan pose
          TimePoint time_point_J =
              ros_data->lidar_container[curr_index + j].time_point;

          if (time_point_J >= next_pose_time) // stop interpolation
          {
            break;
          }

          T_MAP_LLOC_J.matrix() = interpolateTransform(
              T_MAP_LLOC_k.matrix(), curr_pose_time, T_MAP_LLOC_kp1.matrix(),
              next_pose_time, time_point_J);
          T_MAP_LMAP_J.matrix() = interpolateTransform(
              T_MAP_LMAP_k.matrix(), curr_pose_time, T_MAP_LMAP_kp1.matrix(),
              next_pose_time, time_point_J);

          bool take_new_map_scan = takeNewScan(
              T_MAP_LLOC_Jprev, T_MAP_LLOC_J, this->params.map_sampling_dist,
              this->params.map_rotation_change);

          // interpolate pose and add new scan to current target cloud
          if (take_new_map_scan) {
            pcl::transformPointCloud(
                *(ros_data->lidar_container[curr_index + j].value),
                *cloud_interp, T_MAP_LLOC_J);
            *cloud_target += *cloud_interp;
            pcl::transformPointCloud(
                *(ros_data->lidar_container_map[scan_range_map.first + j]
                      .value),
                *cloud_interp, T_MAP_LMAP_J);
            *cloud_target += *cloud_interp;
            T_MAP_LLOC_Jprev = T_MAP_LLOC_J;
            T_MAP_LMAP_Jprev = T_MAP_LMAP_J;
          }
        }
      }
      break;
    }

    // this block makes sure every intermediate map is made up of n scans.
    // each scan is filtered individually when saved in measurements
    // containers, then the whole set of n
    // combined scans is filtered once (default, n=15)
    if ((k % this->params.int_map_size) == 0) {
      // every nth pose, filter the intermediate map if specified then
      // move to next
      if ((i != 0) && !(this->params.downsample_output_method == 3)) {
        // if not first intermediate map, then filter it and
        // move to the next intermediate map
        *intermediaries.at(i) = downSampleFilterIG(
            intermediaries.at(i), this->params.downsample_cell_size);
        i++;
      } else {
        // if first intermediate map
        if (k != 0) {
          // if it's the first scan, do nothing.
          // if it's not the first scan, but it is the first
          // intermediate map , then increase iterator of int. maps
          i++;
        }
      }
      // add new empty point cloud to set of intermediate maps
      intermediaries.emplace_back(new pcl::PointCloud<pcl::PointXYZ>);
    }

    // filter each new cloud if specified in config
    if (this->params.downsample_output_method == 1) {
      *cloud_target = downSampleFilterIG(this->cloud_target,
                                         this->params.downsample_cell_size);
    }

    // add each new cloud to current intermediate map
    *(intermediaries.at(i)) += *this->cloud_target;
  }

  for (uint64_t iter = 0; iter < intermediaries.size(); iter++) {
    *this->aggregate += *(intermediaries.at(iter));
  }
}

void ScanMatcher::outputAggregateMap(int mapping_method, std::string path_) {
  if (this->params.downsample_output_method == 3) {
    *this->aggregate =
        downSampleFilterIG(this->aggregate, this->params.downsample_cell_size);
  }
  std::string dateandtime = convertTimeToDate(std::chrono::system_clock::now());
  std::string mapType;
  switch (mapping_method) {
  case 1:
    mapType = "_loc_map.pcd";
    break;
  case 2:
    mapType = "_map.pcd";
    break;
  case 3:
    mapType = "_comb_map.pcd";
    break;
  }

  pcl::io::savePCDFileBinary(path_ + dateandtime + mapType, *this->aggregate);
  LOG_INFO("outputting map at time: %s", dateandtime.c_str());
}

void ScanMatcher::outputOptTraj(std::string path_) {
  std::ofstream file;
  std::string dateandtime = convertTimeToDate(std::chrono::system_clock::now());
  const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision,
                                         Eigen::DontAlignCols, ", ", ", ");
  file.open(path_ + dateandtime + "_opt_traj" + ".txt");
  for (uint64_t iter = 0; iter < this->final_poses.size(); iter++) {
    file << this->final_poses[iter].time_point.time_since_epoch().count()
         << ", ";
    file << this->final_poses[iter].value.matrix().format(CSVFormat);
    file << std::endl;
  }
  file.close();
}

void ScanMatcher::outputInitTraj(std::string path_) {
  std::ofstream file;
  std::string dateandtime = convertTimeToDate(std::chrono::system_clock::now());
  const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision,
                                         Eigen::DontAlignCols, ", ", ", ");
  file.open(path_ + dateandtime + "_init_traj" + ".txt");
  for (uint64_t iter = 0; iter < this->init_pose.size(); iter++) {
    file << this->init_pose[iter].time_point.time_since_epoch().count() << ", ";
    file << this->init_pose[iter].value.matrix().format(CSVFormat);
    file << std::endl;
  }
  file.close();
}

void ScanMatcher::outputForColourization(boost::shared_ptr<ROSBag> ros_data,
                                         std::string path_) {

  std::string outputPathRoot, outputPathThisCam, outputPathThisCamPCDs,
      outputPathThisCamImgs;

  outputPathRoot = path_ + "/colourization/";

  Eigen::Affine3d T_C_L;
  pcl::PointCloud<pcl::PointXYZ>::Ptr aggregateTransformed(
      new pcl::PointCloud<pcl::PointXYZ>);
  TimePoint poseTimePoint;

  for (uint32_t i = 0; i < this->params.intrinsics.size(); i++) {
    // For each camera, create separate save directories
    outputPathThisCam = outputPathRoot + this->params.intrinsics[i];
    outputPathThisCam.erase(outputPathThisCam.end() - 5,
                            outputPathThisCam.end());
    outputPathThisCamPCDs = outputPathThisCam + "/pcds/";
    outputPathThisCamImgs = outputPathThisCam + "/images/";
    boost::filesystem::create_directories(outputPathThisCamPCDs);
    boost::filesystem::create_directories(outputPathThisCamImgs);

    // Output the intrinsics to results folder
    std::string intrinsicsPath = __FILE__;
    intrinsicsPath.erase(intrinsicsPath.end() - 20, intrinsicsPath.end());
    intrinsicsPath += "calibrations/" + this->params.intrinsics[i];
    std::ifstream src(intrinsicsPath, std::ios::binary);
    std::ofstream dst(outputPathThisCam + "/" + this->params.intrinsics[i],
                      std::ios::binary);
    dst << src.rdbuf();

    // Iterate through poses and save images + transformed clouds
    // TODO: add parameter for how many images are taken
    int mapCounter = 0;
    for (uint32_t i = 0 + 10; i < final_poses.size(); i += 10) {
      mapCounter++;

      // Get transform for pose i. Here L is lidar map frame, and C is
      // camera frame which is also the current pose
      T_C_L = final_poses.at(i).value;

      // transform the aggregate map to the image frame
      pcl::transformPointCloud(*this->aggregate, *aggregateTransformed, T_C_L);
      pcl::io::savePCDFileASCII(outputPathThisCamPCDs + "map" +
                                    std::to_string(mapCounter) + ".pcd",
                                *aggregateTransformed);

      // Get image closest to timestamp of pose i
      poseTimePoint = ros_data->getLidarScanTimePoint(pose_scan_map.at(i));
      ros_data->outputImage(poseTimePoint, outputPathThisCamImgs,
                            this->params.camera_topics[i], mapCounter);
    }
  }
}

// ICPScanMatcher (Child Class) Functions
ICPScanMatcher::ICPScanMatcher(Params &p_, std::string matcherConfigPath)
    : ScanMatcher(p_),
      // segmenter(this->seg_params),
      matcher(wave::ICPMatcherParams(matcherConfigPath)) {
  this->params = p_;
  this->cloud_ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  this->cloud_tgt = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

bool ICPScanMatcher::matchScans(
    uint64_t i, uint64_t j, Eigen::Affine3d &T_Li_Lj, wave::Mat6 &info,
    bool &correction_norm_valid,
    boost::shared_ptr<ROSBag> ros_data) { // j: current, (reference scan)
  // i: adjacent scan (target)
  auto T_MAP_Lj = init_pose.at(j).value; // set initial guess of current scan
  auto T_MAP_Li = init_pose.at(i).value; // set initial guess of adjacent scan
  correction_norm_valid = true;

  // Get scans
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref2(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt2(
      new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ref2 = *ros_data->lidar_container[pose_scan_map.at(j)].value;
  *cloud_tgt2 = *ros_data->lidar_container[pose_scan_map.at(i)].value;
  TimePoint timepoint_j, timepoint_i;

  // Combine scans if specified
  if (this->params.combine_scans) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref_tmp(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_tmp(
        new pcl::PointCloud<pcl::PointXYZ>);
    timepoint_j = ros_data->lidar_container[pose_scan_map.at(j)].time_point;
    timepoint_i = ros_data->lidar_container[pose_scan_map.at(i)].time_point;
    uint64_t map_index_j =
        getLidarTimeWindow(ros_data->lidar_container_map, timepoint_j);
    uint64_t map_index_i =
        getLidarTimeWindow(ros_data->lidar_container_map, timepoint_i);
    pcl::transformPointCloud(
        *(ros_data->lidar_container_map[map_index_j].value), *cloud_ref_tmp,
        this->params.T_LMAP_LLOC.inverse());
    pcl::transformPointCloud(
        *(ros_data->lidar_container_map[map_index_i].value), *cloud_tgt_tmp,
        this->params.T_LMAP_LLOC.inverse());
    *cloud_ref2 += *cloud_ref_tmp;
    *cloud_tgt2 += *cloud_tgt_tmp;
  }

  // set current scan as reference scan
  this->matcher.setRef(cloud_ref2);

  // display reference scan
  this->displayPointCloud(cloud_ref2, 0); // white

  // calculate init. T from adjacent to current
  auto T_estLj_Li = T_MAP_Lj.inverse() * T_MAP_Li;

  // transform adjacent scan (i) to estimated current scan frame using
  // initialized T, then assign to cloud_target
  pcl::transformPointCloud(*cloud_tgt2, *(this->cloud_tgt), T_estLj_Li);

  this->matcher.setTarget(this->cloud_tgt);
  this->displayPointCloud(this->cloud_tgt, 1); // red
  if (matcher.match()) {
    auto T_estLj_Lj = this->matcher.getResult(); // assign estimated transform
                                                 // to new current position
    this->displayPointCloud(cloud_tgt, 2, T_estLj_Lj.inverse()); // blue
    if (this->params.visualize && this->params.step_matches) {
      std::cin.get(); // wait for user to hit next
    }
    double delta = T_estLj_Lj.matrix().block(0, 3, 3, 1).norm();
    double total = T_estLj_Li.matrix().block(0, 3, 3, 1).norm();

    if (this->params.use_gps & ((delta / total) > 0.4)) {
      LOG_INFO("Correction norm %f %%", 100.0 * delta / total);
      correction_norm_valid = false;
      return false;
    }

    T_Li_Lj = T_estLj_Li.inverse() *
              T_estLj_Lj; // correct estimated T by match result

    // try scaling info by how much correction was required
    if (this->params.fixed_scan_transform_cov) {
      info = this->params.scan_transform_cov;
    } else {
      matcher.estimateInfo();
      info = this->matcher.getInfo(); //(0.1 / (0.1 + delta)) *
    }
    return true;
  } else {
    return false;
  }
}

// GICPScanMatcher (Child Class) Functions
GICPScanMatcher::GICPScanMatcher(Params &p_, std::string matcherConfigPath)
    : ScanMatcher(p_), matcher(wave::GICPMatcherParams(matcherConfigPath)) {
  this->params = p_;
  this->cloud_ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  this->cloud_tgt = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

bool GICPScanMatcher::matchScans(
    uint64_t i, uint64_t j, Eigen::Affine3d &T_Li_Lj, wave::Mat6 &info,
    bool &correction_norm_valid,
    boost::shared_ptr<ROSBag> ros_data) { // j: current, (reference scan)
  // i: adjacent scan (target)
  auto T_MAP_Lj = init_pose.at(j).value; // set initial guess of current scan
  auto T_MAP_Li = init_pose.at(i).value; // set initial guess of adjacent scan
  correction_norm_valid = true;

  // Get scans
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref2(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt2(
      new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ref2 = *ros_data->lidar_container[pose_scan_map.at(j)].value;
  *cloud_tgt2 = *ros_data->lidar_container[pose_scan_map.at(i)].value;
  TimePoint timepoint_j, timepoint_i;

  // Combine scans if specified
  if (this->params.combine_scans) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref_tmp(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_tmp(
        new pcl::PointCloud<pcl::PointXYZ>);
    timepoint_j = ros_data->lidar_container[pose_scan_map.at(j)].time_point;
    timepoint_i = ros_data->lidar_container[pose_scan_map.at(i)].time_point;
    uint64_t map_index_j =
        getLidarTimeWindow(ros_data->lidar_container_map, timepoint_j);
    uint64_t map_index_i =
        getLidarTimeWindow(ros_data->lidar_container_map, timepoint_i);
    pcl::transformPointCloud(
        *(ros_data->lidar_container_map[map_index_j].value), *cloud_ref_tmp,
        this->params.T_LMAP_LLOC.inverse());
    pcl::transformPointCloud(
        *(ros_data->lidar_container_map[map_index_i].value), *cloud_tgt_tmp,
        this->params.T_LMAP_LLOC.inverse());
    *cloud_ref2 += *cloud_ref_tmp;
    *cloud_tgt2 += *cloud_tgt_tmp;
  }

  // set current scan as reference scan
  this->matcher.setRef(cloud_ref2);

  // display reference scan
  this->displayPointCloud(cloud_ref2, 0); // white

  // calculate init. T from adjacent to current
  auto T_estLj_Li = T_MAP_Lj.inverse() * T_MAP_Li;

  // transform adjacent scan (i) to estimated current scan frame using
  // initialized T, then assign to cloud_target
  pcl::transformPointCloud(*cloud_tgt2, *(this->cloud_tgt), T_estLj_Li);

  this->matcher.setTarget(this->cloud_tgt);
  this->displayPointCloud(this->cloud_tgt, 1); // red
  if (matcher.match()) {
    auto T_estLj_Lj = this->matcher.getResult(); // assign estimated transform
                                                 // to new current position
    this->displayPointCloud(cloud_tgt, 2, T_estLj_Lj.inverse()); // blue
    if (this->params.visualize && this->params.step_matches) {
      std::cin.get(); // wait for user to hit next
    }
    double delta = T_estLj_Lj.matrix().block(0, 3, 3, 1).norm();
    double total = T_estLj_Li.matrix().block(0, 3, 3, 1).norm();

    if (this->params.use_gps & ((delta / total) > 0.4)) {
      LOG_INFO("Correction norm %f %%", 100.0 * delta / total);
      correction_norm_valid = false;
      return false;
    }

    T_Li_Lj = T_estLj_Li.inverse() *
              T_estLj_Lj; // correct estimated T by match result

    // try scaling info by how much correction was required
    if (this->params.fixed_scan_transform_cov) {
      info = this->params.scan_transform_cov;
    } else {
      matcher.estimateInfo();
      info = this->matcher.getInfo(); //(0.1 / (0.1 + delta)) *
    }
    return true;
  } else {
    return false;
  }
}
