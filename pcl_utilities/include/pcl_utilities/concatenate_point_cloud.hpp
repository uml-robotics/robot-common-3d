#ifndef ROBOT_COMMON_3D_SRC_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_CONCATENATE_POINT_CLOUD_H_
#define ROBOT_COMMON_3D_SRC_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_CONCATENATE_POINT_CLOUD_H_
#include <rclcpp/visibility_control.hpp>  // RCLCPP_EXPORT

#include <pcl_utility_msgs/srv/pcl_concatenate_point_cloud.hpp>  // pcl_utility_msgs::msg::PCLConcatenatePointCloud

namespace pcl_utilities
{
using pcl_utility_msgs::srv::PCLConcatenatePointCloud;

/**
 * @brief Concatenate an array of PointCloud2 into a single PointCloud2.
 *
 * Given an array of PointCloud2 messages, concatenate into a single PointCloud2
 * message.
 *
 * @param[in] req sensor_msgs/PointCloud2[] Container of PointCloud2 messages.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 */
RCLCPP_EXPORT void concatenate_point_cloud(
  const PCLConcatenatePointCloud::Request & req, PCLConcatenatePointCloud::Response & res);
}  // namespace pcl_utilities
#endif
