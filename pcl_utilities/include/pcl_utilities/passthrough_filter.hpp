#ifndef ROBOT_COMMON_3D_SRC_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_PASSTHROUGH_FILTER_H_
#define ROBOT_COMMON_3D_SRC_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_PASSTHROUGH_FILTER_H_
#include "rclcpp/visibility_control.hpp"  // RCLCPP_EXPORT

#include "pcl_utility_msgs/srv/pcl_passthrough_filter.hpp"  // pcl_utility_msgs::srv::PCLPassthroughFilter

namespace pcl_utilities
{
using pcl_utility_msgs::srv::PCLPassthroughFilter;

/**
 * @brief Apply a passthrough (x,y,z) filter to a PointCloud2 message.
 *
 * Given a PointCloud2 message, apply a passthrough (x,y,z) filter and
 * provide the resulting PointCloud2 message. More information about pcl filters
 * at: https://pcl.readthedocs.io/projects/tutorials/en/master/# This filter:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html#passthrough
 *
 * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 */
RCLCPP_EXPORT void passthrough_filter(
  const PCLPassthroughFilter::Request & req, PCLPassthroughFilter::Response & res);
}  // namespace pcl_utilities
#endif
