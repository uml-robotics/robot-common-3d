#ifndef ROBOT_COMMON_3D_SRC_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_VOXEL_GRID_FILTER_H_
#define ROBOT_COMMON_3D_SRC_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_VOXEL_GRID_FILTER_H_
#include <rclcpp/visibility_control.hpp>  // RCLCPP_EXPORT

#include <pcl_utility_msgs/srv/pcl_voxel_grid_filter.hpp>  // pcl_utility_msgs::srv::PCLVoxelGridFilter

namespace pcl_utilities
{
using pcl_utility_msgs::srv::PCLVoxelGridFilter;

/**
 * @brief Downsample a PointCloud2 message by applying a voxelgrid filter.
 *
 * Given a PointCloud2 message, apply a voxelgrid filter and provide the
 * resulting PointCloud2 message. More information about pcl filters at:
 * https://pcl.readthedocs.io/projects/tutorials/en/master/# This filter:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html#voxelgrid
 *
 * @param[in] req sensor_msgs/msg/PointCloud2 A PointCloud2 message.
 * @param[out] res sensor_msgs/msg/PointCloud2 A PointCloud2 message.
 */
RCLCPP_EXPORT void voxel_grid_filter(
  const PCLVoxelGridFilter::Request & req, PCLVoxelGridFilter::Response & res);
}  // namespace pcl_utilities
#endif
