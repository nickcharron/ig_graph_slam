// ROS and other Headers
#include <unistd.h>
#include <sstream>
#include <string>
#include <math.h>
#include <chrono>
#include <ctime>

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// WAVE Headers
#include <wave/utils/log.hpp>
#include <wave/matching/icp.hpp>
#include <wave/matching/pointcloud_display.hpp>

// IG Graph SLAM Headers
#include "gtsam_graph.hpp"
#include "scan_matcher.hpp"
#include "load_ros_data.hpp"
#include "conversions.hpp"
#include "pcl_filters.hpp"
#include "measurementtypes.hpp"
#include "kdtreetype.hpp"

// General Functions
  //void fillparams(boost::shared_ptr<Params> params)
  void fillparams(Params &params)
  {
      wave::ConfigParser parser;
      parser.addParam("bag_file_path", &(params.bag_file_path));
      parser.addParam("lidar_topic_loc", &(params.lidar_topic_loc));
      parser.addParam("lidar_topic_map", &(params.lidar_topic_map));
      parser.addParam("mapping_method", &(params.mapping_method));
      parser.addParam("gps_topic", &(params.gps_topic));
      parser.addParam("gps_imu_topic", &(params.gps_imu_topic));
      parser.addParam("odom_topic", &(params.odom_topic));
      parser.addParam("init_method", &(params.init_method));
      parser.addParam("gps_type", &(params.gps_type));
      parser.addParam("int_map_size", &(params.int_map_size));
      parser.addParam("T_LIDAR_GPS", &(params.T_LIDAR_GPS.matrix()));
      parser.addParam("T_LMAP_LLOC", &(params.T_LMAP_LLOC.matrix()));
      parser.addParam("k_nearest_neighbours", &(params.knn));
      parser.addParam("trajectory_sampling_distance", &(params.trajectory_sampling_dist));
      parser.addParam("map_sampling_distance", &(params.map_sampling_dist));
      parser.addParam("distance_match_limit", &(params.distance_match_limit));
      parser.addParam("distance_match_min", &(params.distance_match_min));
      parser.addParam("x_lower_threshold", &(params.x_lower_threshold));
      parser.addParam("x_upper_threshold", &(params.x_upper_threshold));
      parser.addParam("y_lower_threshold", &(params.y_lower_threshold));
      parser.addParam("y_upper_threshold", &(params.y_upper_threshold));
      parser.addParam("z_lower_threshold", &(params.z_lower_threshold));
      parser.addParam("z_upper_threshold", &(params.z_upper_threshold));
      parser.addParam("use_pass_through_filter", &(params.use_pass_through_filter));
      parser.addParam("x_lower_threshold_map", &(params.x_lower_threshold_map));
      parser.addParam("x_upper_threshold_map", &(params.x_upper_threshold_map));
      parser.addParam("y_lower_threshold_map", &(params.y_lower_threshold_map));
      parser.addParam("y_upper_threshold_map", &(params.y_upper_threshold_map));
      parser.addParam("z_lower_threshold_map", &(params.z_lower_threshold_map));
      parser.addParam("z_upper_threshold_map", &(params.z_upper_threshold));
      parser.addParam("use_pass_through_filter_map", &(params.use_pass_through_filter_map));
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
      parser.addParam("downsample_output_method", &(params.downsample_output_method));
      parser.addParam("iterations", &(params.iterations));
      parser.addParam("visualize", &(params.visualize));
      parser.addParam("step_matches", &(params.step_matches));
      parser.addParam("optimize_gps_lidar", &(params.optimize_gps_lidar));
      parser.addParam("fixed_scan_transform_cov", &(params.fixed_scan_transform_cov));
      parser.addParam("scan_transform_cov", &(params.scan_transform_cov));

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

  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;

// Scan Matcher (parent Class) Functions
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

  void ICP1ScanMatcher::createAggregateMap(GTSAMGraph &graph, boost::shared_ptr<ROSBag> ros_data)
  {
      int i =0;
      for (uint64_t k = 0; k < graph.poses.size(); k++) // NOTE: Sometimes I get seg fault on the last scan
      { // iterate through all poses in graph

        // Define transforms we will need
        Eigen::Affine3d T_MAP_LLOC_k, T_MAP_LLOC_kp1, T_MAP_LMAP_k,
                        T_MAP_LMAP_kp1, T_LMAP_LLOC, T_MAP_LLOC_Jprev, T_MAP_LMAP_Jprev;
        T_MAP_LLOC_k = this->final_poses.at(graph.poses.at(k)).value;  // for pose k
        T_LMAP_LLOC = this->params.T_LMAP_LLOC;                       // static
        T_MAP_LMAP_k = T_MAP_LLOC_k * T_LMAP_LLOC.inverse();

        int curr_index = this->pose_scan_map.at(graph.poses.at(k));
        TimePoint curr_pose_time = ros_data->lidar_container[curr_index].time_point;
        int next_index;
        TimePoint next_pose_time = curr_pose_time;

        // get all time and transforms for next pose for interpolation
        if(!(k == graph.poses.size()-1))
        {
          T_MAP_LLOC_kp1 = this->final_poses.at(graph.poses.at(k+1)).value;  // for pose k + 1
          T_MAP_LMAP_kp1 = T_MAP_LLOC_kp1 * T_LMAP_LLOC.inverse();
          T_MAP_LLOC_Jprev = T_MAP_LLOC_k;
          T_MAP_LMAP_Jprev = T_MAP_LMAP_k;
          next_index = this->pose_scan_map.at(graph.poses.at(k+1));
          next_pose_time = ros_data->lidar_container[next_index].time_point;
        }

        // find scan range for map scans:
        std::pair<int, int> scan_range_map;
        scan_range_map = getLidarTimeWindow(ros_data->lidar_container_map, curr_pose_time, next_pose_time);

        switch(this->params.mapping_method)
        {
          case 1 :
           // transform current pose scan to target cloud
           pcl::transformPointCloud(*(ros_data->lidar_container[curr_index].value),
                                    *this->cloud_target,
                                    T_MAP_LLOC_k);

            // iterate through all scans between pose k and k+1
            if ( (this->params.trajectory_sampling_dist > this->params.map_sampling_dist)
                  && !(k == graph.poses.size()-1) )
            {
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_interp (new pcl::PointCloud<pcl::PointXYZ>);

              int j = 0;
              while (true)
              {
                j++;
                Eigen::Affine3d T_MAP_LLOC_J;  // interpolated scan pose
                TimePoint time_point_J = ros_data->lidar_container[curr_index+j].time_point;

                if( time_point_J >= next_pose_time) // stop interpolation
                {break;}

                T_MAP_LLOC_J.matrix()  = interpolateTransform(T_MAP_LLOC_k.matrix(), curr_pose_time,
                                                            T_MAP_LLOC_kp1.matrix(), next_pose_time,
                                                            time_point_J);

                bool take_new_map_scan = takeNewScan(T_MAP_LLOC_Jprev, T_MAP_LLOC_J, this->params.map_sampling_dist);

                // interpolate pose and add new scan to current target cloud
                if(take_new_map_scan)
                {
                  pcl::transformPointCloud( *(ros_data->lidar_container[curr_index+j].value),
                                            *cloud_interp,
                                            T_MAP_LLOC_J);
                  *cloud_target += *cloud_interp;
                  T_MAP_LLOC_Jprev = T_MAP_LLOC_J;
                }
              }
            }
            break;

          case 2 :
            // transform current pose scan to target cloud
            pcl::transformPointCloud(*(ros_data->lidar_container_map[scan_range_map.first].value),
                                    *this->cloud_target,
                                    T_MAP_LMAP_k);

            // iterate through all scans between pose k and k+1
            if ( (this->params.trajectory_sampling_dist > this->params.map_sampling_dist)
                  && !(k == graph.poses.size()-1) )
            {
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_interp (new pcl::PointCloud<pcl::PointXYZ>);

              int j = 0;
              while (true)
              {
                j++;
                Eigen::Affine3d T_MAP_LMAP_J;  // interpolated scan pose
                TimePoint time_point_J = ros_data->lidar_container_map[scan_range_map.first+j].time_point;

                if( time_point_J >= next_pose_time) // stop interpolation
                {break;}

                T_MAP_LMAP_J.matrix()  = interpolateTransform(T_MAP_LMAP_k.matrix(), curr_pose_time,
                                                            T_MAP_LMAP_kp1.matrix(), next_pose_time,
                                                            time_point_J);
                bool take_new_map_scan = takeNewScan(T_MAP_LMAP_Jprev, T_MAP_LMAP_J, this->params.map_sampling_dist);

                // interpolate pose and add new scan to current target cloud
                if(take_new_map_scan)
                {
                  pcl::transformPointCloud( *(ros_data->lidar_container_map[scan_range_map.first+j].value),
                                            *cloud_interp,
                                            T_MAP_LMAP_J);
                  *cloud_target += *cloud_interp;
                  T_MAP_LMAP_Jprev = T_MAP_LMAP_J;
                }

              }
            }

            break;
          case 3 :

            //transform current map scan to tmp cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*(ros_data->lidar_container_map[scan_range_map.first].value),
                                     *cloud_tmp,
                                     T_MAP_LMAP_k);
            //transform current localization scan to target cloud
            pcl::transformPointCloud(*(ros_data->lidar_container[curr_index].value),
                                     *this->cloud_target,
                                     T_MAP_LLOC_k);
            //Add tmp cloud to target cloud
             *this->cloud_target+=*cloud_tmp;

             // iterate through all scans between pose k and k+1
             if ( (this->params.trajectory_sampling_dist > this->params.map_sampling_dist)
                   && !(k == graph.poses.size()-1) )
             {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_interp (new pcl::PointCloud<pcl::PointXYZ>);
                int j = 0;
                while (true)
                {
                  j++;
                  Eigen::Affine3d T_MAP_LLOC_J, T_MAP_LMAP_J;  // interpolated scan pose
                  TimePoint time_point_J = ros_data->lidar_container[curr_index+j].time_point;

                  if( time_point_J >= next_pose_time) // stop interpolation
                  {break;}

                  T_MAP_LLOC_J.matrix()  = interpolateTransform(T_MAP_LLOC_k.matrix(), curr_pose_time,
                                                              T_MAP_LLOC_kp1.matrix(), next_pose_time,
                                                              time_point_J);
                  T_MAP_LMAP_J.matrix()  = interpolateTransform(T_MAP_LMAP_k.matrix(), curr_pose_time,
                                                              T_MAP_LMAP_kp1.matrix(), next_pose_time,
                                                              time_point_J);

                  bool take_new_map_scan = takeNewScan(T_MAP_LLOC_Jprev, T_MAP_LLOC_J, this->params.map_sampling_dist);

                  // interpolate pose and add new scan to current target cloud
                  if(take_new_map_scan)
                  {
                    pcl::transformPointCloud( *(ros_data->lidar_container[curr_index+j].value),
                                              *cloud_interp,
                                              T_MAP_LLOC_J);
                    *cloud_target += *cloud_interp;
                    pcl::transformPointCloud( *(ros_data->lidar_container_map[scan_range_map.first+j].value),
                                              *cloud_interp,
                                              T_MAP_LMAP_J);
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
          if ((k % this->params.int_map_size) == 0)
          { // every nth pose, filter the intermediate map if specified then move to next
              if ((i != 0) && !(this->params.downsample_output_method ==3) )
              {   // if not first intermediate map, then filter it and
                  // move to the next intermediate map
                  *this->intermediaries.at(i) = downSampleFilterIG(this->intermediaries.at(i), this->params.downsample_cell_size);
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

          // filter each new cloud if specified in config
          if (this->params.downsample_output_method == 1)
          {
              *cloud_target = downSampleFilterIG(this->cloud_target, this->params.downsample_cell_size);
          }

          // add each new cloud to current intermediate map
          *(this->intermediaries.at(i)) += *this->cloud_target;
      }

      for (uint64_t iter = 0; iter < this->intermediaries.size(); iter++)
      {
          *this->aggregate += *(this->intermediaries.at(iter));
      }

  }

  void ICP1ScanMatcher::outputAggregateMap(GTSAMGraph &graph, boost::shared_ptr<ROSBag> ros_data)
  {
      *this->aggregate = downSampleFilterIG(this->aggregate, this->params.downsample_cell_size);
      long timestamp = std::chrono::system_clock::now().time_since_epoch().count();
      pcl::io::savePCDFileBinary(std::to_string(timestamp) + "aggregate_map.pcd", *this->aggregate);
      std::cout << "outputting map at time: " << std::to_string(timestamp) << std::endl;

      //now save trajectory to file
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

      // TODO: Make this work even without GPS data
      if(this->params.init_method == 1)
      {
        // This file contains the input traj to the GTSAM since we want to compare
        // the pose difference betweem the input and the output of GTSAM
        std::ofstream gt_file;
        gt_file.open(std::to_string(timestamp) + "GTSAMinputTraj.txt");
        for (uint64_t j = 0; j < adjacency->size(); j++)
        {
            Eigen::Affine3d T_ECEF_GPSIMU = ros_data->getGPSTransform(ros_data->getLidarScanTimePoint(pose_scan_map.at(j)), true);
            Eigen::Affine3d T_MAP_GPSIMU, T_MAP_LIDAR;
            T_MAP_GPSIMU = ros_data->T_ECEF_MAP.inverse() * T_ECEF_GPSIMU;
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
            gt_file << ros_data->getLidarScanTimePoint(pose_scan_map.at(j)).time_since_epoch().count() << ", ";
            gt_file << T_MAP_LIDAR.matrix().format(CSVFormat);
            gt_file << std::endl;
        }
        gt_file.close();

        std::ofstream datumfile;
        using dbl = std::numeric_limits<double>;
        datumfile.open(std::to_string(timestamp) + "map_ecef_datum" + ".txt");
        datumfile.precision(dbl::max_digits10);
        datumfile << ros_data->T_ECEF_MAP.matrix().format(CSVFormat);
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

  }

// ICP1ScanMatcher (Child Class) Functions
  ICP1ScanMatcher::ICP1ScanMatcher(Params &p_)
      : ScanMatcher(p_),
        //segmenter(this->seg_params),
        matcher(wave::ICPMatcherParams(p_.matcher_config))
  {
      this->params = p_;

      // this->centre.setZero();
      // this->bounds << 1.8, 1.8;
      // this->groundCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      // this->obsCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      // this->drvCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      this->pcl_pc2 = boost::make_shared<pcl::PCLPointCloud2>();
      this->cloud_ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      this->cloud_target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      this->aggregate = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  void ICP1ScanMatcher::createPoseScanMap(boost::shared_ptr<ROSBag> ros_data)
  {
      LOG_INFO("Storing pose scans...");
      // save initial poses of the lidar scans based on GPS data and save iterators
      // corresponding to these scans
      int i = 0;
      Eigen::Affine3d T_ECEF_GPS, T_MAP_LIDAR;
      for (uint64_t iter = 0; iter < ros_data->lidar_container.size(); iter++)
      {  // this ierates through the lidar measurements
          bool use_next_scan = false;
          switch(this->params.init_method)
          {
            case 1 :
              try
              {
                  // extract gps measurement at the same timepoint as the current lidar message
                  auto gps_pose = ros_data->gps_container.get(ros_data->lidar_container[iter].time_point, 0);
                  T_ECEF_GPS = gpsToEigen(gps_pose.first, true); // true: apply T_ENU_GPS
                  T_MAP_LIDAR =  ros_data->T_ECEF_MAP.inverse() * T_ECEF_GPS * this->params.T_LIDAR_GPS.inverse();
              }
              catch (const std::out_of_range &e)
              {
                  LOG_INFO("No gps pose for time of scan, may happen at edges of recorded data");
                  use_next_scan = true;
                  break;
              }
            case 2 :
              try
              {
                  // extract odometry pose at the same timepoint as current lidar message
                  auto odom_pose = ros_data->odom_container.get(ros_data->lidar_container[iter].time_point, 2);
                  T_MAP_LIDAR = odom_pose.first;
              }
              catch (const std::out_of_range &e)
              {
                  LOG_INFO("No odometry message for time of scan, may happen at edges of recorded data");
                  use_next_scan = true;
                  break;
              }
          }

          // If i > 0 then check to see if the distance between current scan and
          // last scan is greater than the minimum, if so then save this pose
          // If i = 0, then save the scan - first scan
          if ( i > 0 )
          {
            bool take_new_scan;
            take_new_scan = takeNewScan(T_MAP_LIDAR, init_pose.poses[i - 1],
                                    this->params.trajectory_sampling_dist);
            if (take_new_scan && !use_next_scan)
            {
              this->init_pose.poses.push_back(T_MAP_LIDAR);
              this->pose_scan_map.push_back(iter);
              ++i;
            }
          }
          else if (!use_next_scan)
          {
            this->init_pose.poses.push_back(T_MAP_LIDAR);
            this->pose_scan_map.push_back(iter);
            ++i;
          }
      }
      LOG_INFO("Stored %d pose scans of %d available scans.", i, ros_data->lidar_container.size());
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

  bool ICP1ScanMatcher::matchScans(uint64_t i, uint64_t j, Eigen::Affine3d &T_Li_Lj, wave::Mat6 &info, bool &correction_norm_valid, boost::shared_ptr<ROSBag> ros_data)
  { // j: current, (reference scan)
    // i: adjacent scan (target)
    auto T_MAP_Lj = init_pose.poses[j]; // set initial guess of current scan
    auto T_MAP_Li = init_pose.poses[i]; // set initial guess of adjacent scan
    correction_norm_valid = true;
    // set current scan as reference scan
    this->matcher.setRef(ros_data->lidar_container[pose_scan_map.at(j)].value);
    // display reference scan
    this->displayPointCloud(ros_data->lidar_container[pose_scan_map.at(j)].value, 0); // white
    auto T_estLj_Li = T_MAP_Lj.inverse() * T_MAP_Li; // calculate init. T from adjacent to current
    pcl::transformPointCloud( *(ros_data->lidar_container[pose_scan_map.at(i)].value),
                              *(this->cloud_target),
                              T_estLj_Li );
                              // transform adjacent scan (i) to estimated
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
