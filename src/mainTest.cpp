//#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <limits>
#include <Eigen/Core>

#include <test2.hpp>
//
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
// #include <ros/ros.h>
// #include <ros/time.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <novatel_msgs/INSPVAX.h>
//
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/filters/conditional_removal.h>
//
// #include <wave/containers/measurement_container.hpp>
// #include <wave/utils/math.hpp>
// #include <wave/utils/config.hpp>
// #include <wave/utils/log.hpp>
// #include <wave/matching/icp.hpp>
// #include <wave/matching/gicp.hpp>
// #include <wave/matching/pointcloud_display.hpp>
// #include <wave/matching/ground_segmentation.hpp>
// #include <gtsam_graph.hpp>

//#include "kdtreetype.hpp"
//#include "offline_matcher.hpp"

// const uint64_t bias_offset = 1000000;
// using Clock = std::chrono::steady_clock;
// using TimePoint = std::chrono::time_point<Clock>;

int main() {
    std::cout << "Test Succesful!" << std::endl;

    TESTNODE result;
    int b = 4;
    result.test(b);
}
