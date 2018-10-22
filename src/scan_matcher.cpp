// ROS and other Headers
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <novatel_msgs/INSPVAX.h>
#include <unistd.h>
#include <sstream>
#include <string>
#include <math.h>

// WAVE Headers
#include <wave/containers/measurement_container.hpp>
#include <wave/utils/math.hpp>
#include <wave/utils/config.hpp>
#include <wave/utils/log.hpp>
#include <wave/matching/icp.hpp>
#include <wave/matching/gicp.hpp>
#include <wave/matching/pointcloud_display.hpp>
#include <wave/matching/ground_segmentation.hpp>

// IG Graph SLAM Headers
#include "gtsam_graph.hpp"
#include "scan_matcher.hpp"
#include "conversions.hpp"
#include "measurementtypes.hpp"
#include "kdtreetype.hpp"

int pointCloudMsgCount = 0, gpsMsgCount = 0, imuMsgCount = 0;
double initial_heading = 0;

// General Functions
  void fillparams(Params &params)
  {
      wave::ConfigParser parser;
      parser.addParam("bag_file_path", &(params.bag_file_path));
      parser.addParam("lidar_topic", &(params.lidar_topic));
      parser.addParam("gps_topic", &(params.gps_topic));
      parser.addParam("gps_imu_topic", &(params.gps_imu_topic));
      parser.addParam("gps_type", &(params.gps_type));
      parser.addParam("k_nearest_neighbours", &(params.knn));
      parser.addParam("trajectory_sampling_distance", &(params.trajectory_sampling_dist));
      parser.addParam("distance_match_limit", &(params.distance_match_limit));
      parser.addParam("distance_match_min", &(params.distance_match_min));
      parser.addParam("x_lower_threshold", &(params.x_lower_threshold));
      parser.addParam("x_upper_threshold", &(params.x_upper_threshold));
      parser.addParam("y_lower_threshold", &(params.y_lower_threshold));
      parser.addParam("y_upper_threshold", &(params.y_upper_threshold));
      parser.addParam("z_lower_threshold", &(params.z_lower_threshold));
      parser.addParam("z_upper_threshold", &(params.z_upper_threshold));
      parser.addParam("use_pass_through_filter", &(params.use_pass_through_filter));
      parser.addParam("downsample_input", &(params.downsample_input));
      parser.addParam("input_downsample_size", &(params.input_downsample_size));
      parser.addParam("use_rad_filter", &(params.use_rad_filter));
      parser.addParam("set_min_neighbours", &(params.set_min_neighbours));
      parser.addParam("set_search_radius", &(params.set_search_radius));
      parser.addParam("matcher_type", &(params.matcher_type));
      parser.addParam("matcher_config_path", &(params.matcher_config));
      parser.addParam("ground_segment", &(params.ground_segment));
      parser.addParam("use_gps", &(params.use_gps));
      parser.addParam("downsample_cell_size", &(params.downsample_cell_size));
      parser.addParam("iterations", &(params.iterations));
      parser.addParam("visualize", &(params.visualize));
      parser.addParam("step_matches", &(params.step_matches));
      parser.addParam("optimize_gps_lidar", &(params.optimize_gps_lidar));
      parser.addParam("fixed_scan_transform_cov", &(params.fixed_scan_transform_cov));
      parser.addParam("scan_transform_cov", &(params.scan_transform_cov));

      parser.addParam("T_LIDAR_GPS", &(params.T_LIDAR_GPS.matrix()));

      //TODO find a better way to specify the path to config file
      /*
        char cwd[PATH_MAX];
        stringstream ss;
        string cwd_s;
        ss << cwd;
        ss >> cwd_s;
        parser.load(cwd_s);
      */
      parser.load("/home/nick/ig_catkin_ws/src/ig_graph_slam/config/ig_graph_slam_config.yaml");
      ++params.knn;  // Nearest neighbour search returns same point, so increment
      // might as well square these now
      // Why are these squared? Changed to x and y threshold
      params.distance_match_limit *= params.distance_match_limit;
      params.distance_match_min *= params.distance_match_min;
  }

  bool takeNewScan(const Eigen::Affine3d &p1,
                   const Eigen::Affine3d &p2,
                   const double &dist)
  {
      // calculate the norm of the distance between the two points
      double l2sqrd = (p1(0, 3) - p2(0, 3)) * (p1(0, 3) - p2(0, 3)) +
                      (p1(1, 3) - p2(1, 3)) * (p1(1, 3) - p2(1, 3)) +
                      (p1(2, 3) - p2(2, 3)) * (p1(2, 3) - p2(2, 3));

      // if the norm is greater than the specified minimum sampling distance
      if (l2sqrd > dist * dist)
      {
        // then yes take a new scan
          return true;
      }
      else
      {
        // then no, do not take a new scan
          return false;
      }
  }

// Scan Matcher (parent Class) Functions
  void ScanMatcher::loadGPSDataFromNavSatFix(boost::shared_ptr<sensor_msgs::NavSatFix> gps_msg)
  {
    // get rpy from imu container:
    TimePoint imu_msg_time = rosTimeToChrono(gps_msg->header);
    auto rpy_msg = this->imu_container.get(imu_msg_time, 0); // (vector3, vector3)

    wave::Vec6 gps_measurement; // LLA, RPY
    gps_measurement << gps_msg->latitude,
                       gps_msg->longitude,
                       gps_msg->altitude,
                       rpy_msg.first(0), // roll
                       rpy_msg.first(1), // pitch
                       rpy_msg.first(2); // yaw

    wave::Vec6 gps_stdev;
    gps_stdev << sqrt(gps_msg->position_covariance[0]),
                 sqrt(gps_msg->position_covariance[4]),
                 sqrt(gps_msg->position_covariance[8]),
                 rpy_msg.second(0), // roll stddev
                 rpy_msg.second(1), // pitch stddev
                 rpy_msg.second(2); // yaw stddev

    // This sets your initial transform from map frame to earth-centered earth-fixed frame
    if (!this->have_GPS_datum)
    {
      this->have_GPS_datum = true;
      this->T_ECEF_MAP = gpsToEigen(gps_measurement, false);
    }
    this->gps_container.emplace(
          rosTimeToChrono(gps_msg->header),
          0,
          std::make_pair(gps_measurement, gps_stdev)); // (Vector6, Vector6)
  }

  void ScanMatcher::loadGPSDataFromINSPVAX(boost::shared_ptr<novatel_msgs::INSPVAX> gps_msg)
  {
      wave::Vec6 gps_measurement; // LLA, RPY
      gps_measurement << gps_msg->latitude,
                         gps_msg->longitude,
                         gps_msg->altitude + gps_msg->undulation,
                         gps_msg->roll * DEG_TO_RAD,
                         gps_msg->pitch * DEG_TO_RAD,
                         -gps_msg->azimuth * DEG_TO_RAD;
      wave::Vec6 gps_stdev;
      gps_stdev << gps_msg->latitude_std,
                   gps_msg->longitude_std,
                   gps_msg->altitude_std,
                   gps_msg->roll_std * DEG_TO_RAD,
                   gps_msg->pitch_std * DEG_TO_RAD,
                   gps_msg->azimuth_std * DEG_TO_RAD;

      if (!this->have_GPS_datum)
      {
          this->have_GPS_datum = true;
          this->T_ECEF_MAP = gpsToEigen(gps_measurement, false);
      }

      this->gps_container.emplace(
              gpsTimeToChrono(gps_msg->header.gps_week,
                              gps_msg->header.gps_week_seconds),
              0,
              std::make_pair(gps_measurement, gps_stdev)); // (Vector6, Vector6)
  }

  void ScanMatcher::findAdjacentScans()
  {
     kd_tree_t index(3, this->init_pose, nanoflann::KDTreeSingleIndexAdaptorParams(10));

     index.buildIndex();
     std::vector<size_t> ret_indices(this->params.knn);
     std::vector<double> out_dist_sqr(this->params.knn);

     // Scan registration class
     // As factors are added, factor id will just be counter value
     this->adjacency = boost::make_shared<std::vector<std::vector<uint64_t>>>(this->init_pose.poses.size());

     /*
      * For each pose within the initial pose, get the position.
      * Connect all the positions until the last pose by adding it to the adjacency
      * at that point
      *
      */
     for (uint64_t j = 0; j < this->init_pose.poses.size(); j++)
     {
       // query is the position of the current pose
        double query[3] = {this->init_pose.poses[j](0, 3),
                           this->init_pose.poses[j](1, 3),
                           this->init_pose.poses[j](2, 3)};

       // Do this for all poses except the last one
        if (j + 1 < this->init_pose.poses.size())
        {  // ensures that trajectory is connected
          this->adjacency->at(j).emplace_back(j + 1);
          this->total_matches++;
        }

       // Search for the specified amount of nearest neighbours to the position
       // within the original init_pose poses
         index.knnSearch(query, this->params.knn, &ret_indices[0], &out_dist_sqr[0]);

       // For each pose position, check the nearest neighbours if they are
       // within the boundaries specified
       // If they are and they are after j, add them to the adjacency
       // If the points were discovered before j, they won't be added to
       // the adjacency
         for (uint16_t k = 0; k < this->params.knn; k++)
         {
           if ((out_dist_sqr[ret_indices[k]] > this->params.distance_match_min) &&
               (out_dist_sqr[ret_indices[k]] < this->params.distance_match_limit))
           {
             if (j < ret_indices[k])
             {
               // add indices of K nearest neighbours to back of vector for scan j
               this->adjacency->at(j).emplace_back(ret_indices[k]);
               this->total_matches++;
             }
           }
         }
     }
  }

  Eigen::Affine3d ScanMatcher::getGPSTransform(const TimePoint &time_point, bool applyT_ENU_GPS)
  {
      return gpsToEigen(this->gps_container.get(time_point, 0).first, applyT_ENU_GPS);
  }

  void ICP1ScanMatcher::createAggregateMap(GTSAMGraph &graph)
  {
      // set leaf size for voxel grid filter
      this->downsampler.setLeafSize(this->params.downsample_cell_size,
                                    this->params.downsample_cell_size,
                                    this->params.downsample_cell_size);
      int i = 0;
      for (uint64_t k = 0; k < graph.poses.size(); k++)
      { // iterate through all poses in graph
          // transform current scan to target cloud using T_Map_Pk
          pcl::transformPointCloud(*(this->lidar_container[this->pose_scan_map.at(graph.poses.at(k))].value),
                                   *this->cloud_target,
                                   this->final_poses.at(graph.poses.at(k)).value);

          // this block makes sure every intermediate map is made up of 15 scans.
          // each scan is filtered individually, then the whole set of 15
          // combined scans is filtered once
          // TODO: Make this and all other filtering a param in the yaml.
          if ((k % 15) == 0)
          { // every 15th pose, filter the intermediate map
              if (i != 0) // if not first intermediate map, then filter it and
              {           // move to the next intermediate map
                  this->downsampler.setInputCloud(this->intermediaries.at(i));
                  this->downsampler.filter(*(this->intermediaries.at(i)));
                  i++;
              }
              else // if first intermediate map
              {
                  if (k != 0)
                  { // if it's the first scan, do nothing.
                    // if it's not the first scan, but it is the first
                    // intermediate map , then increase iterator of int. maps
                      i++;
                  }
              }
              // add new empty point cloud to set of intermediate maps
              this->intermediaries.emplace_back(new pcl::PointCloud<pcl::PointXYZ>);
          }

          // filter each new cloud
          this->downsampler.setInputCloud(this->cloud_target);
          this->downsampler.filter(*this->cloud_target);
          // add each new cloud to current intermediate map
          *(this->intermediaries.at(i)) += *this->cloud_target;
      }

      for (uint64_t iter = 0; iter < this->intermediaries.size(); iter++)
      {
          // TODO: Add option to visualize the map building
          *this->aggregate += *(this->intermediaries.at(iter));
      }
  }

  void ICP1ScanMatcher::outputAggregateMap(GTSAMGraph &graph)
  {
      this->downsampler.setLeafSize(this->params.downsample_cell_size,
                                    this->params.downsample_cell_size,
                                    this->params.downsample_cell_size);
      this->downsampler.setInputCloud(this->aggregate);
      this->downsampler.filter(*this->aggregate);
      long timestamp = std::chrono::system_clock::now().time_since_epoch().count();
      pcl::io::savePCDFileBinary(std::to_string(timestamp) + "aggregate_map.pcd", *this->aggregate);
      std::cout << "outputting map at time: " << std::to_string(timestamp) << std::endl;

      // now save trajectory to file
      std::ofstream file;
      const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ");
      file.open(std::to_string(timestamp) + "opt_traj" + ".txt");
      for (auto iter = final_poses.begin(); iter != final_poses.end(); iter++)
      {
          file << iter->time_point.time_since_epoch().count() << ", ";
          file << iter->value.matrix().format(CSVFormat);
          file << std::endl;
      }
      file.close();

      std::ofstream bias_file;
      bias_file.open(std::to_string(timestamp) + "GPSbias.txt");
      for (auto iter = graph.biases.begin(); iter != graph.biases.end(); iter++)
      {
          auto curbias = graph.result.at<gtsam::Point3>(*iter);
          bias_file << curbias.x() << ", " << curbias.y() << ", " << curbias.z() << std::endl;
      }
      bias_file.close();

      // This file contains the input traj to the GTSAM since we want to compare
      // the pose difference betweem the input and the output of GTSAM
      std::ofstream gt_file;
      gt_file.open(std::to_string(timestamp) + "GTSAMinputTraj.txt");
      for (uint64_t j = 0; j < adjacency->size(); j++)
      {
          Eigen::Affine3d T_ECEF_GPSIMU = getGPSTransform(getLidarScanTimePoint(pose_scan_map.at(j)), true);
          Eigen::Affine3d T_MAP_GPSIMU, T_MAP_LIDAR;
          T_MAP_GPSIMU = T_ECEF_MAP.inverse() * T_ECEF_GPSIMU;
          if (this->params.optimize_gps_lidar)
          {
              auto result = graph.result.at<gtsam::Pose3>(6000000);
              T_MAP_LIDAR = T_MAP_GPSIMU * result.matrix();
          }
          else
          {
              auto result = params.T_LIDAR_GPS.inverse();
              T_MAP_LIDAR = T_MAP_GPSIMU * result.matrix();
          }
          gt_file << getLidarScanTimePoint(pose_scan_map.at(j)).time_since_epoch().count() << ", ";
          gt_file << T_MAP_LIDAR.matrix().format(CSVFormat);
          gt_file << std::endl;
      }
      gt_file.close();

      std::ofstream datumfile;
      using dbl = std::numeric_limits<double>;
      datumfile.open(std::to_string(timestamp) + "map_ecef_datum" + ".txt");
      datumfile.precision(dbl::max_digits10);
      datumfile << T_ECEF_MAP.matrix().format(CSVFormat);
      datumfile.close();
      if (this->params.optimize_gps_lidar)
      {
          auto result = graph.result.at<gtsam::Pose3>(6000000);
          std::ofstream datumfile;
          using dbl = std::numeric_limits<double>;
          datumfile.open(std::to_string(timestamp) + "T_LIDAR_GPS" + ".txt");
          datumfile.precision(dbl::max_digits10);
          datumfile << result.inverse().matrix().format(CSVFormat);
          datumfile.close();
      }
  }

// ICP1ScanMatcher (Child Class) Functions
  ICP1ScanMatcher::ICP1ScanMatcher(Params &p_)
      : ScanMatcher(p_),
        //segmenter(this->seg_params),
        matcher(wave::ICPMatcherParams(p_.matcher_config))
  {
      this->params = p_;
      this->pass_filter_x.setFilterFieldName("x");
      this->pass_filter_y.setFilterFieldName("y");
      this->pass_filter_z.setFilterFieldName("z");
      this->pass_filter_x.setFilterLimits(this->params.x_lower_threshold, this->params.x_upper_threshold);
      this->pass_filter_y.setFilterLimits(this->params.y_lower_threshold, this->params.y_upper_threshold);
      this->pass_filter_z.setFilterLimits(this->params.z_lower_threshold, this->params.z_upper_threshold);
      // this->centre.setZero();
      // this->bounds << 1.8, 1.8;
      // this->groundCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      // this->obsCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      // this->drvCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      this->radfilter.setMinNeighborsInRadius(this->params.set_min_neighbours);
      this->radfilter.setRadiusSearch(this->params.set_search_radius);
      this->pcl_pc2 = boost::make_shared<pcl::PCLPointCloud2>();
      this->cloud_ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      this->cloud_target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      this->aggregate = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  void ICP1ScanMatcher::loadIMUMessage(rosbag::View::iterator &rosbag_iter, bool end_of_bag, bool start_of_bag)
  {
      if (rosbag_iter->getTopic() == this->params.gps_imu_topic)
      {
          imuMsgCount++;
          auto imu_msg = rosbag_iter->instantiate<geometry_msgs::Vector3Stamped>();

          if (start_of_bag)
          {
            initial_heading = -imu_msg->vector.z;
          }

          wave::Vec6 rpy_measurement;
          rpy_measurement << -imu_msg->vector.y, // roll
                             imu_msg->vector.x - 1.570796327, // pitch (starts at pi/2)
                             -imu_msg->vector.z - initial_heading, // yaw
                             0, // cannot use Vec3 in libwave for
                             0, // measurement containters
                             0;
          wave::Vec6 rpy_stdev;
          rpy_stdev << 10,  // TODO: get real std dev from imu data
                       10,
                       100,
                       0,
                       0,
                       0;
          this->imu_container.emplace(
                rosTimeToChrono(imu_msg->header),
                0,
                std::make_pair(rpy_measurement, rpy_stdev)); // (Vector3, Vector3)
      }
      if(end_of_bag)
      {
        LOG_INFO("Saved %d IMU Messages.", imuMsgCount);
      }
  }

  void ICP1ScanMatcher::loadROSBagMessage(rosbag::View::iterator &rosbag_iter, bool end_of_bag)
  {
      if (rosbag_iter->getTopic() == this->params.gps_topic)
      {
          gpsMsgCount++;
          auto gps_msg_navsatfix = rosbag_iter->instantiate<sensor_msgs::NavSatFix>();
          auto gps_msg_inspvax = rosbag_iter->instantiate<novatel_msgs::INSPVAX>();
          if (this->params.gps_type == "NavSatFix")
          {
            this->loadGPSDataFromNavSatFix(gps_msg_navsatfix);
          }
          else if (this->params.gps_type == "INSPVAX")
          {
            this->loadGPSDataFromINSPVAX(gps_msg_inspvax);
          }
          else
          {
            LOG_ERROR("Improper gps_type entered in config file. input: %s. Use NavSatFix or INSPVAX.", this->params.gps_type);
          }

      }
      else if (rosbag_iter->getTopic() == this->params.lidar_topic)
      {
          pointCloudMsgCount++;
          auto lidar_msg = rosbag_iter->instantiate<sensor_msgs::PointCloud2>();
          this->loadPCLPointCloudFromPointCloud2(lidar_msg);
      }
      if(end_of_bag)
      {
        LOG_INFO("Saved %d GPS Messages.", gpsMsgCount);
        LOG_INFO("Saved %d Point Cloud Messages.", pointCloudMsgCount);
      }
  }

  void ICP1ScanMatcher::loadPCLPointCloudFromPointCloud2(boost::shared_ptr<sensor_msgs::PointCloud2> lidar_msg)
  {
      pcl_conversions::toPCL(*lidar_msg, *this->pcl_pc2);
      pcl::fromPCLPointCloud2(*this->pcl_pc2, *this->cloud_ref);

      // check this, it might not be implemented correctly! See how CropXY is used
      if (this->params.use_pass_through_filter)
      {
          this->pass_filter_x.setInputCloud(this->cloud_ref);
          this->pass_filter_x.filter(*this->cloud_ref);
          this->pass_filter_y.setInputCloud(this->cloud_ref);
          this->pass_filter_y.filter(*this->cloud_ref);
          this->pass_filter_z.setInputCloud(this->cloud_ref);
          this->pass_filter_z.filter(*this->cloud_ref);
      }
      //cropXY(this->centre, this->bounds, this->cloud_ref, this->cloud_ref, true);

      // ***Add ground segmenter here?

      if (this->params.downsample_input)
      {
          this->downsampler.setLeafSize(this->params.input_downsample_size,
                                        this->params.input_downsample_size,
                                        this->params.input_downsample_size);
          this->downsampler.setInputCloud(this->cloud_ref);
          this->downsampler.filter(*this->cloud_ref);
      }

      if(this->params.use_rad_filter)
      {
        this->radfilter.setInputCloud(this->cloud_ref);
        this->radfilter.filter(*this->cloud_ref);
      }

      wave::PCLPointCloudPtr temp = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      *temp = *this->cloud_ref;
      this->lidar_container.emplace_back(rosTimeToChrono(lidar_msg->header), 0, temp);
  }

  void ICP1ScanMatcher::createPoseScanMap()
  {
      // save initial poses of the lidar scans based on GPS data and save iterators
      // corresponding to these scans
      int i = 0;
      Eigen::Affine3d T_ECEF_GPS, T_MAP_LIDAR;
      for (uint64_t iter = 0; iter < this->lidar_container.size(); iter++)
      {  // this ierates through the lidar measurements
        //std::cout << "Hit 'Enter' to continue" << std::endl;
        //std::cin.get(); // wait for user to hit next
          try
          {
              // extract gps measurement at the same timepoint as the current lidar message
              auto gps_pose = this->gps_container.get(this->lidar_container[iter].time_point, 0);
              T_ECEF_GPS = gpsToEigen(gps_pose.first, true); // true: apply T_ENU_GPS
                  // TODO: check this ^ might need to change this to false
              T_MAP_LIDAR =  this->T_ECEF_MAP.inverse() * T_ECEF_GPS * this->params.T_LIDAR_GPS.inverse();

              // If i > 0 then check to see if the distance between current scan and
              // last scan is greater than the minimum, if so then save this pose
              // If i = 0, then save the scan - first scan
              if ( i > 0 )
              {
                bool take_new_scan;
                take_new_scan = takeNewScan(T_MAP_LIDAR, init_pose.poses[i - 1],
                                        this->params.trajectory_sampling_dist);
                if (take_new_scan)
                {
                  this->init_pose.poses.push_back(T_MAP_LIDAR);
                  this->pose_scan_map.push_back(iter);
                  ++i;
                  LOG_INFO("Stored scan pose %d of %d available.", i, this->lidar_container.size());
                }
              }
              else
              {
                this->init_pose.poses.push_back(T_MAP_LIDAR);
                this->pose_scan_map.push_back(iter);
                ++i;
              }
          }
          catch (const std::out_of_range &e)
          {
              LOG_INFO("No gps pose for time of scan, may happen at edges of recorded data");
          }
      }
  }

  TimePoint ICP1ScanMatcher::getLidarScanTimePoint(int index)
  {
      return this->lidar_container[index].time_point;
  }

  void ICP1ScanMatcher::displayPointCloud(wave::PCLPointCloudPtr cloud_display,
                                    int color, const Eigen::Affine3d &transform)
  {
    if (this->params.visualize)
    {
        *this->cloud_temp_display = *cloud_display;
        if (!transform.matrix().isIdentity())
        {
            pcl::transformPointCloud(*cloud_display, *this->cloud_temp_display, transform);
        }
        this->init_display.addPointcloud(this->cloud_temp_display, color);
    }
  }

  bool ICP1ScanMatcher::matchScans(uint64_t i, uint64_t j, Eigen::Affine3d &T_Li_Lj, wave::Mat6 &info, bool &correction_norm_valid)
  { // j: current, (reference scan)
    // i: adjacent scan (target)
    auto T_MAP_Lj = init_pose.poses[j]; // set initial guess of current scan
    auto T_MAP_Li = init_pose.poses[i]; // set initial guess of adjacent scan
    correction_norm_valid = true;
    // set current scan as reference scan
    this->matcher.setRef(this->lidar_container[pose_scan_map.at(j)].value);
    // display reference scan
    this->displayPointCloud(this->lidar_container[pose_scan_map.at(j)].value, 0); // white
    auto T_estLj_Li = T_MAP_Lj.inverse() * T_MAP_Li; // calculate init. T from adjacent to current
    pcl::transformPointCloud( *(lidar_container[pose_scan_map.at(i)].value),
                              *(this->cloud_target),
                              T_estLj_Li );
                              // transform adjacent scan (i) to estimate
                              // current scan frame using initialized T, then
                              // assign to cloud_target
    this->matcher.setTarget(cloud_target);
    this->displayPointCloud(cloud_target, 1); // red
    if (matcher.match())
    {
        auto T_estLj_Lj = this->matcher.getResult(); // assign estimated transform to new current position
        this->displayPointCloud(cloud_target, 2, T_estLj_Lj.inverse()); // blue
        if (this->params.visualize && this->params.step_matches)
        {
            std::cin.get(); // wait for user to hit next
        }

        double delta = T_estLj_Lj.matrix().block(0, 3, 3, 1).norm();
        double total = T_estLj_Li.matrix().block(0, 3, 3, 1).norm();

        if (this->params.use_gps & ((delta / total) > 0.4))
        {
            LOG_INFO("Correction norm %f %%", 100.0 * delta / total);
            correction_norm_valid = false;
            return false;
        }

        T_Li_Lj = T_estLj_Li.inverse() * T_estLj_Lj; // correct estimated T by match result

        // try scaling info by how much correction was required
        if (this->params.fixed_scan_transform_cov)
        {
            info = this->params.scan_transform_cov;
        }
        else
        {
            matcher.estimateInfo();
            info = this->matcher.getInfo();  //(0.1 / (0.1 + delta)) *
        }
        return true;
    }
    else
    {
        return false;
    }
  }
