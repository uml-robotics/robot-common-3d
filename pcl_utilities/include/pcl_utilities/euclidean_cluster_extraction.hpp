#ifndef ROBOT_COMMON_3D_SRC_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_EUCLIDEAN_CLUSTER_EXTRACTION_H_
#define ROBOT_COMMON_3D_SRC_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_EUCLIDEAN_CLUSTER_EXTRACTION_H_
#include "rclcpp/visibility_control.hpp"  // RCLCPP_EXPORT

#include "pcl_utility_msgs/srv/pcl_euclidean_cluster_extraction.hpp"  // pcl_utility_msgs::srv::PCLEuclideanClusterExtraction

namespace pcl_utilities
{
using pcl_utility_msgs::srv::PCLEuclideanClusterExtraction;

/**
 * @brief Segment clusters within a PointCloud into individual cloud objects.
 *
 * Given a PointCloud2 message, segment clusters of points into their
 * own PointCloud2 objects for further processing/handling.
 *
 * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 */
RCLCPP_EXPORT void euclidean_cluster_extraction(
  const PCLEuclideanClusterExtraction::Request & req,
  PCLEuclideanClusterExtraction::Response & res);
}  // namespace pcl_utilities
#endif
