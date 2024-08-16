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

#include <cstdint>  // std::int64_t
#include <functional>  // std::bind
#include <iomanip>  // std::quoted
#include <memory>  // std::make_shared
#include <sstream>  // std::stringstream
#include <string>
#include <utility>  // std::move

#include "pcl_utility_msgs/srv/pcl_transform_point_cloud.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"  // tf2::doTransform

using pcl_utility_msgs::srv::PCLTransformPointCloud;

class TransformPointCloudService : public rclcpp::Node
{
private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Service<PCLTransformPointCloud>::SharedPtr
    transform_pointcloud_service_;
  int64_t max_transforms_;

public:
  TransformPointCloudService()
  : rclcpp::Node("transform_point_cloud_service"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    // -1 signals an infinite number of attempts
    // If a value is set to above this, it should probably be -1
    constexpr int64_t MAX_REASONABLE_TRANSFORM_ATTEMPTS = 500;

    // Setup ROS parameters
    rcl_interfaces::msg::ParameterDescriptor param_constraints;
    rcl_interfaces::msg::IntegerRange integer_range;

    integer_range.from_value = -1;
    integer_range.to_value = MAX_REASONABLE_TRANSFORM_ATTEMPTS;
    param_constraints.integer_range.push_back(integer_range);

    max_transforms_ = declare_parameter<int64_t>(
      "max_transform_attempts", std::move(param_constraints));

    // Attach the callback
    auto callback = std::bind(
      &TransformPointCloudService::transformm_point_cloud,
      this, std::placeholders::_1, std::placeholders::_2);

    transform_pointcloud_service_ = create_service<PCLTransformPointCloud>(
      "transform_point_cloud", std::move(callback));
  }

  /**
   * Transform an array of PointCloud2 into a single PointCloud2.
   *
   * Given an array of PointCloud2 messages, transform into a single PointCloud2
   * message.
   *
   * @param[in] req sensor_msgs/PointCloud2 Container of PointCloud2 messages.
   * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool transformm_point_cloud(
    PCLTransformPointCloud::Request::SharedPtr req,
    PCLTransformPointCloud::Response::SharedPtr res)
  {
    get_parameter<int64_t>("max_transform_attempts", max_transforms_);

    geometry_msgs::msg::TransformStamped transform;
    bool got_transform = false;

    // if the user picks -1, it will attempt a near infinite amount of times
    for (int64_t runs = 0; runs < max_transforms_; runs++) {
      try {
        transform = tf_buffer_.lookupTransform(
          req->target_frame, req->cloud_in.header.frame_id, tf2::TimePointZero);

        got_transform = true;
        break;
      } catch (const tf2::TransformException &) {}
    }

    if (!got_transform) {
      RCLCPP_ERROR(
        get_logger(), "No transform recieved from %s to %s",
        req->target_frame.c_str(), req->cloud_in.header.frame_id.c_str());

      return false;
    }

    tf2::doTransform(req->cloud_in, res->cloud_out, transform);

    return true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TransformPointCloudService>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
