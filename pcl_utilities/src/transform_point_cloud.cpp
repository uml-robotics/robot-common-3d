/*
 * Author: Daniel Maccaline
 * Date: Apr 1, 2024
 * Editors: Christian Tagliamonte
 * Last Modified: Aug 16, 2024
 * Adapted from:
 * https://github.com/uml-robotics/SpotCommonBehaviors/commits/main/widget_detect_and_grab/include/widget_detect_and_grab/WidgetDetector.hpp
 *
 * Description: Starts up a service for transforming point cloud messages.
 *
 * Input: sensor_msgs/PointCloud2
 * Output: sensor_msgs/PointCloud2
 *
 * Usage:
 *    `ros2 launch pcl_utilities transform_point_cloud.xml`
 */
#include <cstdint>    // uint32_t
#include <stdexcept>  // std::runtime_error

#include "rclcpp/clock.hpp"  // rclcpp::Clock

#include "sensor_msgs/msg/point_cloud2.hpp"  // sensor_msgs::msg::PointCloud2

#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"  // tf2::doTransform

#include "pcl_utilities/transform_point_cloud.hpp"

namespace pcl_utilities
{
void PointCloudTransformer::operator()(
  const PCLTransformPointCloud::Request & req, PCLTransformPointCloud::Response & res) const
{
  geometry_msgs::msg::TransformStamped transform;
  bool got_transform = false;

  for (uint32_t runs = 0; runs < req.max_transform_attempts; runs++) {
    try {
      transform =
        buffer_.lookupTransform(req.target_frame, req.cloud_in.header.frame_id, tf2::TimePointZero);

      got_transform = true;
      break;
    } catch (const tf2::TransformException &) {
    }
  }

  if (!got_transform) {
    std::stringstream stream;
    stream << "No transform recieved from" << req.target_frame << " to "
           << req.cloud_in.header.frame_id.c_str();

    throw std::runtime_error(stream.str());
  }

  tf2::doTransform(req.cloud_in, res.cloud_out, transform);
}

}  // namespace pcl_utilities

#ifndef PCL_UTILITIES_IS_LIBRARY
#include <rclcpp/utilities.hpp>

#include "pcl_utilities/detail/param.hpp"
#include "pcl_utilities/detail/service_runner.hpp"

int main(int argc, char ** argv)
{
  using SrvType = pcl_utilities::PCLTransformPointCloud;
  constexpr auto service_name = "transform_point_cloud";
  constexpr auto node_namespace = "robot_common_3d/pcl_utilities";

  rclcpp::init(argc, argv);
  pcl_utilities::ServiceRunner<SrvType> service_runner(service_name, node_namespace);
  pcl_utilities::PointCloudTransformer transformer{service_runner.get_node()->get_clock()};
  service_runner.define_service(service_name, std::ref(transformer));
  service_runner.expose_request_parameters(
    [](auto & request) {
      return std::tuple{
        pcl_utilities::Param("max_transform_attempts", &request.max_transform_attempts)};
    });
  service_runner.spin_multi_thread();

  rclcpp::shutdown();

  return 0;
}
#endif
