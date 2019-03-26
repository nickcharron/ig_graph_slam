#ifndef IG_GRAPH_SLAM_PCL_FILTERS_HPP
#define IG_GRAPH_SLAM_PCL_FILTERS_HPP

// PCL Headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

// WAVE headers
#include <wave/matching/icp.hpp>

// libbeam specific headers
#include <beam/utils/math.hpp>

inline pcl::PointCloud<pcl::PointXYZ> passThroughFilterIG(wave::PCLPointCloudPtr cloud_in_, beam::Vec6 threshold_)
{
  pcl::PassThrough<pcl::PointXYZ> pass_filter_x, pass_filter_y, pass_filter_z;
  pass_filter_x.setFilterFieldName("x");
  pass_filter_y.setFilterFieldName("y");
  pass_filter_z.setFilterFieldName("z");

  pass_filter_x.setFilterLimits(threshold_[0], threshold_[1]);
  pass_filter_y.setFilterLimits(threshold_[2], threshold_[3]);
  pass_filter_z.setFilterLimits(threshold_[4], threshold_[5]);

  pass_filter_x.setInputCloud(cloud_in_);
  pass_filter_x.filter(*cloud_in_);
  pass_filter_y.setInputCloud(cloud_in_);
  pass_filter_y.filter(*cloud_in_);
  pass_filter_z.setInputCloud(cloud_in_);
  pass_filter_z.filter(*cloud_in_);
  return *cloud_in_;
}

inline pcl::PointCloud<pcl::PointXYZ> downSampleFilterIG(wave::PCLPointCloudPtr cloud_in_, float downsample_size_)
{
  pcl::VoxelGrid<pcl::PointXYZ> downsampler;

  downsampler.setLeafSize(downsample_size_,
                          downsample_size_,
                          downsample_size_);

  downsampler.setInputCloud(cloud_in_);
  downsampler.filter(*cloud_in_);
  return *cloud_in_;
}

inline pcl::PointCloud<pcl::PointXYZ> radFilterIG(wave::PCLPointCloudPtr cloud_in_, float min_nei, float sr)
{
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radfilter;
  radfilter.setMinNeighborsInRadius(min_nei);
  radfilter.setRadiusSearch(sr);
  radfilter.setInputCloud(cloud_in_);
  radfilter.filter(*cloud_in_);
  return *cloud_in_;
}

#endif // IG_GRAPH_SLAM_PCL_FILTERS_HPP
