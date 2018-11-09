#ifndef IG_GRAPH_SLAM_PCL_FILTERS_HPP
#define IG_GRAPH_SLAM_PCL_FILTERS_HPP

// PCL Headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

// WAVE headers
#include <wave/utils/math.hpp>
#include <wave/matching/icp.hpp>

pcl::PointCloud<pcl::PointXYZ> passThroughFilterIG(wave::PCLPointCloudPtr cloud_in_, wave::Vec6 threshold_);

pcl::PointCloud<pcl::PointXYZ> downSampleFilterIG(wave::PCLPointCloudPtr cloud_in_, float downsample_size_);

pcl::PointCloud<pcl::PointXYZ> radFilterIG(wave::PCLPointCloudPtr cloud_in_, float min_nei, float sr);

#endif // IG_GRAPH_SLAM_PCL_FILTERS_HPP
