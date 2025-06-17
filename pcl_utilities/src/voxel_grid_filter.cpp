/*
 * Author: Brian Flynn
 * Date: Nov 15, 2022
 * Editors: Christian Tagliamonte
 * Last Modified: Aug 5, 2024
 * Adapted from:
 * https://github.com/uml-robotics/armada_behaviors/blob/main/armada_flexbe_utilities/src/service/pcl_voxel_grid_filter_service.cpp
 *
 * Description: Starts up a service for a PCL voxel grid filtering.
 *
 * Input: sensor_msgs/PointCloud2
 * Output: sensor_msgs/PointCloud2
 *
 * Usage:
 *    `ros2 launch pcl_utilities voxel_grid_filter.xml`
 */
#include "pcl/filters/voxel_grid.h"  // pcl::VoxelGrid
#include "pcl/memory.h"
#include "pcl_conversions/pcl_conversions.h"  // fromROSMsg, toROSMSG

#include "sensor_msgs/msg/point_cloud2.hpp" // sensor_msgs::msg::PointCloud2

#include "pcl_utilities/voxel_grid_filter.hpp"

namespace pcl_utilities
{
/*
TODO: Check that the leaf size is large enough for the number of points.
      Unit test will pass when the
PCL Error:
  `Leaf size is too small for the input dataset. Integer indices would
overflow.`

Test Failure:
  ```
  [simple_test_voxel_grid_filter-2] [ERROR] [1721769013.741232423]
[tests.robot_common_3d.pcl_utilities.simple_test_voxel_grid_filter]: Sample at
time: 1721769013.7074 produces a pointcloud with more points than the input.
  ```
The custom test fails only when PCL produces an error message.
The output cloud is likely being padded with more points in this case,
causing the number of output clouds be greater than the input cloud.
*/

void voxel_grid_filter(const PCLVoxelGridFilter::Request & req, PCLVoxelGridFilter::Response & res)
{
  pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
  auto input_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  pcl::fromROSMsg(req.cloud_in, *input_cloud);

  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setInputCloud(input_cloud);
  vox.setLeafSize(req.leaf_size_x, req.leaf_size_y, req.leaf_size_z);
  vox.filter(filtered_cloud);

  pcl::toROSMsg(filtered_cloud, res.cloud_out);
}
}  // namespace pcl_utilities

#ifndef PCL_UTILITIES_IS_LIBRARY
#include "rclcpp/utilities.hpp"

#include "pcl_utilities/detail/param.hpp"
#include "pcl_utilities/detail/service_runner.hpp"

int main(int argc, char ** argv)
{
  using SrvType = pcl_utilities::PCLVoxelGridFilter;
  constexpr auto service_name = "voxel_grid_filter";
  constexpr auto node_namespace = "robot_common_3d/pcl_utilities";

  rclcpp::init(argc, argv);
  pcl_utilities::ServiceRunner<SrvType> service_runner(service_name, node_namespace);
  service_runner.define_service(service_name, pcl_utilities::voxel_grid_filter);
  service_runner.expose_request_parameters(
    [](auto & request) {
      std::string prefix{"filters.voxel_grid."};

      return std::tuple{
        pcl_utilities::Param(prefix + "leaf_size_x", &request.leaf_size_x),
        pcl_utilities::Param(prefix + "leaf_size_y", &request.leaf_size_y),
        pcl_utilities::Param(prefix + "leaf_size_z", &request.leaf_size_z)};
    });
  service_runner.spin_multi_thread();
  rclcpp::shutdown();

  return 0;
}
#endif
