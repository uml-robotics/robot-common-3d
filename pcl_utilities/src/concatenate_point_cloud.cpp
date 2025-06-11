/*
 * Author: Brian Flynn
 * Date: Nov 15, 2022
 * Editors: Christian Tagliamonte
 * Last Modified: Aug 4, 2024
 * Adapted from:
 * https://github.com/uml-robotics/armada_behaviors/blob/main/armada_flexbe_utilities/src/service/pcl_concatenate_pointcloud_service.cpp
 *
 * Description: Starts up a service for concatenating point cloud messages.
 *
 * Input: sensor_msgs/PointCloud2[]
 * Output: sensor_msgs/PointCloud2
 *
 * Usage:
 *    `ros2 launch pcl_utilities concatenate_point_cloud.xml`
 */

#include "pcl_utilities/concatenate_point_cloud.hpp"

#include <cstddef>  // size_t
#include <sstream>  // std::string_stream
#include <stdexcept>  // std::runtime_error
#include <string>  // std::string

#include <pcl/point_cloud.h>  // pcl::PointCloud
#include <pcl/point_types.h>  // pcl::PointXYZRGB

#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg, toROSMsg

namespace pcl_utilities
{
void concatenate_point_cloud(
  const PCLConcatenatePointCloud::Request & req, PCLConcatenatePointCloud::Response & res)
{
  // header.frame_id must be managed separately since PCL::PointCloud
  // does not track frame_id
  // Assert that all input_clouds have the same frame_id
  std::string frame_id;
  for (size_t i = 0; i < req.cloud_list_in.size(); ++i) {
    const std::string current_frame_id = req.cloud_list_in[i].header.frame_id;

    if (frame_id == "") {
      frame_id = current_frame_id;
    }

    if (frame_id != current_frame_id) {
      // NOTE: RCLCPP_* macros are threadsafe
      // header.frame_id is not tracked in PCL::PointCloud, must
      // be managed separetely by ROS for concatenation
      // NOTE: The linter only approves of the format below
      std::stringstream error_message;

      error_message << "The point cloud at index #" << i << " has a frame_id of \""
                    << current_frame_id << "\" which does not match the "
                    << "the required frame_id of \"" << frame_id << "\"";

      throw std::runtime_error(error_message.str());
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> concatenated_cloud;

  for (auto & point_cloud : req.cloud_list_in) {
    // req.cloud_list_in[i] becomes invalidated
    pcl::fromROSMsg(point_cloud, input_cloud);
    // PointCloud::operator+= manages is_dense and timestamp feilds
    concatenated_cloud += input_cloud;
  }

  // If there are no point clouds, the output point cloud
  // has a frame id of "" and a default constructed point cloud
  // No function to move from pcl::PointCloud -> PointCloud2
  pcl::toROSMsg(concatenated_cloud, res.cloud_out);
  res.cloud_out.header.frame_id = frame_id;
}

}  // namespace pcl_utilities

#ifndef PCL_UTILITIES_IS_LIBRARY
#include <rclcpp/utilities.hpp>  // rclcpp::init, rclcpp::shutdown

#include "pcl_utilities/detail/param.hpp"
#include "pcl_utilities/detail/service_runner.hpp"

int main(int argc, char ** argv)
{
  using SrvType = pcl_utilities::PCLConcatenatePointCloud;
  constexpr auto service_name = "concatenate_point_cloud";

  rclcpp::init(argc, argv);
  pcl_utilities::ServiceRunner<SrvType> service_runner(service_name, "pcl_utilties");
  service_runner.define_service(service_name, pcl_utilities::concatenate_point_cloud);
  service_runner.spin_one_thread();
  rclcpp::shutdown();

  return 0;
}
#endif
