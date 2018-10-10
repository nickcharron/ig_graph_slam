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
//#include "kdtreetype.hpp"

int pointCloudMsgCount = 0, gpsMsgCount = 0;

void fillparams(Params &params)
{
    wave::ConfigParser parser;
    parser.addParam("bag_file_path", &(params.bag_file_path));
    parser.addParam("lidar_topic", &(params.lidar_topic));
    parser.addParam("gps_topic", &(params.gps_topic));
    parser.addParam("k_nearest_neighbours", &(params.knn));
    parser.addParam("trajectory_sampling_distance", &(params.trajectory_sampling_dist));
    parser.addParam("max_range", &(params.max_range));
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
    params.max_range_neg = -params.max_range;
    // might as well square these now
    // Why are these squared? Changed to x and y threshold
    //params.distance_match_limit *= params.distance_match_limit;
    //params.distance_match_min *= params.distance_match_min;
}

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
