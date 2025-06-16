/*
 * Author: Brian Flynn
 * Date: Nov 17, 2022
 * Editors: Christian Tagliamonte
 * Last Modified: Aug 19, 2024
 * Adapted from:
 * https://github.com/uml-robotics/armada_behaviors/commits/main/armada_flexbe_utilities/src/service/pcl_passthrough_filter_service.cpp
 *
 * Description: Starts up a service for filtering point cloud data with a
 * bounding box.
 *
 * Input: sensor_msgs/PointCloud2
 * Output: sensor_msgs/PointCloud2
 *
 * Usage:
 *    `ros2 launch pcl_utilities passthrough_filter.xml`
 */
#include <memory>  // std::make_shared
#include <utility>  //  std::pair, std::tuple

#include "pcl/filters/passthrough.h"  // pcl::PassthroughFilter
#include "pcl/point_cloud.h"  // pcl::PointCloud
#include "pcl/point_types.h"  // pcl::PointXYZRGB
#include "pcl_conversions/pcl_conversions.h"  // fromROSMsg, toROSMsg

#include "pcl_utilities/passthrough_filter.hpp"

namespace pcl_utilities
{
void passthrough_filter(
  const PCLPassthroughFilter::Request & req, PCLPassthroughFilter::Response & res)
{
  auto temp_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  std::string frame_id = req.cloud_in.header.frame_id;
  fromROSMsg(req.cloud_in, *temp_cloud);

  pcl::PassThrough<pcl::PointXYZRGB> pass_x;
  pass_x.setInputCloud(temp_cloud);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(req.x_min, req.x_max);
  pass_x.filter(*temp_cloud);

  pcl::PassThrough<pcl::PointXYZRGB> pass_y;
  pass_y.setInputCloud(temp_cloud);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(req.y_min, req.y_max);
  pass_y.filter(*temp_cloud);

  pcl::PassThrough<pcl::PointXYZRGB> pass_z;
  pass_z.setInputCloud(temp_cloud);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(req.z_min, req.z_max);
  pass_z.filter(*temp_cloud);

  // preseve the TF frame in the output
  pcl::toROSMsg(*temp_cloud, res.cloud_out);
  res.cloud_out.header.frame_id = frame_id;
}
}  // namespace pcl_utilities

#ifndef PCL_UTILITIES_IS_LIBRARY
#include "rclcpp/utilities.hpp"

#include "pcl_utilities/detail/service_runner.hpp"

int main(int argc, char ** argv)
{
  using SrvType = pcl_utilities::PCLPassthroughFilter;
  constexpr auto service_name = "passthrough_filter";
  constexpr auto node_namespace = "robot_common_3d/pcl_utilities";

  rclcpp::init(argc, argv);

  pcl_utilities::ServiceRunner<SrvType> service_runner(service_name, node_namespace);
  service_runner.define_service(service_name, pcl_utilities::passthrough_filter);
  service_runner.expose_request_parameters(
    [](auto & request) {
      std::string prefix = "filters.passthrough.";

      return std::tuple{
        pcl_utilities::Param(prefix + "x_min", &request.x_min),
        pcl_utilities::Param(prefix + "x_max", &request.x_min),
        pcl_utilities::Param(prefix + "y_min", &request.y_min),
        pcl_utilities::Param(prefix + "y_max", &request.y_max),
        pcl_utilities::Param(prefix + "z_min", &request.z_min),
        pcl_utilities::Param(prefix + "z_max", &request.z_max)};
    });
  service_runner.spin_multi_thread();

  rclcpp::shutdown();

  return 0;
}
#endif
