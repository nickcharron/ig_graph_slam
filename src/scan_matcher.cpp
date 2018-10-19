// ROS and other Headers
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
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

int pointCloudMsgCount = 0, gpsMsgCount = 0;

// General Functions
  void fillparams(Params &params)
  {
      wave::ConfigParser parser;
      parser.addParam("bag_file_path", &(params.bag_file_path));
      parser.addParam("lidar_topic", &(params.lidar_topic));
      parser.addParam("gps_topic", &(params.gps_topic));
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
    wave::Vec6 gps_measurement; // LLA, RPY
    gps_measurement << gps_msg->latitude,
                       gps_msg->longitude,
                       gps_msg->altitude,
                       // We do not have RPY, so set to zero
                       0,
                       0,
                       0;
                       //TODO: Add RPY from IMU onboard GPS
                       // -> see https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros#installation-and-configuration
                       //gps_msg->roll * DEG_TO_RAD,
                       //gps_msg->pitch * DEG_TO_RAD,
                       //-gps_msg->azimuth * DEG_TO_RAD;

    wave::Vec6 gps_stdev;
    gps_stdev << sqrt(gps_msg->position_covariance[0]),
                 sqrt(gps_msg->position_covariance[4]),
                 sqrt(gps_msg->position_covariance[8]),
                 1000000,
                 1000000,
                 1000000;
                 // gps_msg->roll_std * DEG_TO_RAD,
                 // gps_msg->pitch_std * DEG_TO_RAD,
                 // gps_msg->azimuth_std * DEG_TO_RAD;

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
      // this->cloud_target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      // this->aggregate = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  void ICP1ScanMatcher::loadROSBagMessage(rosbag::View::iterator &rosbag_iter, bool end_of_bag)
  {
      if (rosbag_iter->getTopic() == this->params.gps_topic)
      {
          gpsMsgCount++;
          auto gps_msg = rosbag_iter->instantiate<sensor_msgs::NavSatFix>();
          this->loadGPSDataFromNavSatFix(gps_msg);
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
                }
              }
              else
              {
                this->init_pose.poses.push_back(T_MAP_LIDAR);
                this->pose_scan_map.push_back(iter);
                ++i;
              }
              LOG_INFO("Stored scan pose %d of %d.", i, this->lidar_container.size());
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
    this->displayPointCloud(this->lidar_container[pose_scan_map.at(j)].value, 0);

    auto T_estLj_Li = T_MAP_Lj.inverse() * T_MAP_Li; // calculate init. T from adjacent to current

    pcl::transformPointCloud( *(lidar_container[pose_scan_map.at(i)].value),
                              *(this->cloud_target),
                              T_estLj_Li );
                              // transform adjacent scan (i) to estimate
                              // current scan frame using initialized T, then
                              // assign to cloud_target
    this->matcher.setTarget(cloud_target);

    this->displayPointCloud(cloud_target, 1);

    if (matcher.match())
    {
        auto T_estLj_Lj = this->matcher.getResult(); // assign estimated transform to new current position
        this->displayPointCloud(cloud_target, 2, T_estLj_Lj.inverse());

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
