/*
 * Author: Brian Flynn
 * Date: Nov 15, 2022
 * Editors: Christian Tagliamonte
 * Last Modified: Jan 2, 2025
 * Adapted from:
 * https://github.com/uml-robotics/armada_behaviors/blob/main/armada_flexbe_utilities/src/service/pcl_concatenate_pointcloud_service.cpp
 *
 * Description: Starts up a service for extracting point clusters
 *   from an input point cloud message. Thus script returns a list of
 *   individual point cloud messages for each identified cluster.
 *
 * Input: sensor_msgs/PointCloud2
 * Output: sensor_msgs/PointCloud2[]
 *
 * Usage:
 *    `ros2 launch pcl_utilities concatenate_point_cloud.xml`
 */
#include "pcl_utilities/euclidean_cluster_extraction.hpp"

#include <utility>  // std::move
#include <vector>   // std::vector

#include <pcl/memory.h>  // pcl::make_shared<T, Args...>
#include <pcl/point_cloud.h>  // pcl::PointCloud<T>
#include <pcl/point_types.h>  // pcl::PointXYZRGB
#include <pcl/segmentation/extract_clusters.h>  // pcl::EuclideanClusterExtraction
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg, toROSMsg

#include <sensor_msgs/msg/point_cloud2.hpp>  // sensor_msgs::msg::PointCloud2

#include "pcl_utilities/detail/numeric_utils.hpp"  // detail::narrowing_cast

namespace pcl_utilities
{
void euclidean_cluster_extraction(
  const PCLEuclideanClusterExtraction::Request & req, PCLEuclideanClusterExtraction::Response & res)
{
  auto input_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  sensor_msgs::msg::PointCloud2 temp_cloud;
  std::vector<sensor_msgs::msg::PointCloud2> obstacle_cloud_list_out;

  pcl::fromROSMsg(req.cloud_in, *input_cloud);

  auto tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
  tree->setInputCloud(input_cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  std::vector<pcl::PointIndices> cluster_indices;

  ec.setClusterTolerance(req.cluster_tolerance);
  ec.setMinClusterSize(detail::narrowing_cast<pcl::uindex_t>(req.min_cluster_size));
  ec.setMaxClusterSize(detail::narrowing_cast<pcl::uindex_t>(req.max_cluster_size));
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_cloud);
  ec.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_cluster;

  for (const pcl::PointIndices & indecies : cluster_indices) {
    cloud_cluster.clear();

    for (pcl::index_t index : indecies.indices) {
      cloud_cluster.push_back((*input_cloud)[detail::narrowing_cast<size_t>(index)]);
    }

    cloud_cluster.width = detail::narrowing_cast<pcl::uindex_t>(cloud_cluster.size());
    cloud_cluster.height = 1U;
    cloud_cluster.is_dense = true;
    cloud_cluster.header.frame_id = input_cloud->header.frame_id;

    pcl::toROSMsg(cloud_cluster, temp_cloud);
    res.cloud_list_out.push_back(std::move(temp_cloud));
  }
}

}  // namespace pcl_utilities

#ifndef PCL_UTILITIES_IS_LIBRARY
#include <rclcpp/utilities.hpp>

#include "pcl_utilities/detail/param.hpp"
#include "pcl_utilities/detail/service_runner.hpp"

int main(int argc, char ** argv)
{
  using SrvType = pcl_utilities::PCLEuclideanClusterExtraction;
  constexpr auto service_name = "euclidean_cluster_extraction";

  rclcpp::init(argc, argv);
  pcl_utilities::ServiceRunner<SrvType> runner(service_name);
  runner.define_service(service_name, pcl_utilities::euclidean_cluster_extraction);
  runner.expose_request_parameters(
    [](auto & request) {
      std::string prefix = "filters.euclidean_cluster_extraction.";

      return std::tuple{
        pcl_utilities::Param(prefix + "min_cluster_size", &request.min_cluster_size),
        pcl_utilities::Param(prefix + "max_cluster_size", &request.max_cluster_size),
        pcl_utilities::Param(prefix + "cluster_tolerance", &request.cluster_tolerance)};
    });
  runner.spin_multi_thread();

  rclcpp::shutdown();

  return 0;
}
#endif
