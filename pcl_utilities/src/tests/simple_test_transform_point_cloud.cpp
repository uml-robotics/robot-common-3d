/*
 * Author: Christian Tagliamonte
 * Date: Aug Aug 16, 2024
 * Editors: N/A
 * Last Modified: Aug 19, 2024
 *
 * Description: A node to test the concatenate_point_cloud_service.
 * This node listens a point cloud message, then transforms the frame of the
 * point cloud, and publishes the resulting topic.
 * This script takes in two parameters:
 *    `source_frame_name` = The frame to transform from
 *    `point_cloud_topic` = The topic name to read point cloud images from
 * Note: The target TF frame is dynamically generated and not controlled
 * by a parameter.
 *
 * The transformed point cloud is then published to the topic
 *   `transform_point_cloud/cloud_transformed`
 *
 * Usage:
 *    `ros2 launch pcl_utilities test_transform_point_cloud.xml point_cloud_topic:=<POINT_CLOUD_TOPIC>`
 */
#include <chrono>   // std::chrono::seconds
#include <cstdint>  // int64_t
#include <functional>  // std::bind, std::placeholders
#include <ios>  // std::fixed, std::setprecision
#include <memory>  // std::make_shared
#include <optional>  // std::optional
#include <random>
#include <sstream>  // std::stringstream
#include <string>
#include <utility>  // std::move

#include "rclcpp/executors.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include "pcl_utility_msgs/srv/pcl_transform_point_cloud.hpp"

using pcl_utility_msgs::srv::PCLTransformPointCloud;
using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::TransformStamped;

constexpr std::chrono::seconds MAX_WAIT_TIME {1U};

class TestTransformPointCloudNode : public rclcpp::Node
{
private:
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Client<PCLTransformPointCloud>::SharedPtr
    transform_point_cloud_client_;
  rclcpp::Publisher<PointCloud2>::SharedPtr output_publisher_;
  std::string camera_topic_;
  std::string target_frame_name_;
  std::string source_frame_name_;

public:
  TestTransformPointCloudNode()
  : rclcpp::Node("simple_test_transform_point_cloud"),
    tf_broadcaster_(*static_cast<rclcpp::Node *>(this))
  {
    std::string client_topic = declare_parameter<std::string>("node_client_name");
    source_frame_name_ = declare_parameter<std::string>("source_frame_name");
    camera_topic_ = declare_parameter<std::string>("point_cloud_topic");

    transform_point_cloud_client_ =
      create_client<PCLTransformPointCloud>(client_topic);

    output_publisher_ = create_publisher<PointCloud2>(
      "transform_point_cloud/cloud_transformed", 1);

    target_frame_name_ = std::string(get_name()) + "_tf_frame";
  }

  void spin()
  {
    while (rclcpp::ok()) {
      PointCloud2 point_cloud_message;
      bool was_retrieved = rclcpp::wait_for_message(
        point_cloud_message, shared_from_this(),
        camera_topic_, MAX_WAIT_TIME);

      if (!was_retrieved) {
        RCLCPP_ERROR_STREAM(
          get_logger(),
          "A camera message could not be retrieved within 1 second.");
        continue;
      }

      process_point_cloud(std::move(point_cloud_message));
    }
  }

  void process_point_cloud(PointCloud2 && point_cloud)
  {
    // Send a recent transform for the service to use
    TransformStamped tf_transform;
    tf_transform.header.frame_id = source_frame_name_;
    tf_transform.child_frame_id = target_frame_name_;
    tf_transform.header.stamp = get_clock()->now();

    tf_transform.transform.rotation.w = 1.0;
    tf_transform.transform.translation.x = 5.0;

    tf_broadcaster_.sendTransform(std::move(tf_transform));

    auto request = std::make_shared<PCLTransformPointCloud::Request>();
    request->target_frame = target_frame_name_;
    request->cloud_in = point_cloud;

    auto response_future =
      transform_point_cloud_client_->async_send_request(request);

    auto response_code = rclcpp::spin_until_future_complete(
      shared_from_this(), response_future, MAX_WAIT_TIME);

    if (response_code != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to recieve a response from the service");
      return;
    }

    PCLTransformPointCloud::Response::SharedPtr response {
      response_future.get()};
    output_publisher_->publish(response->cloud_out);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestTransformPointCloudNode>();
  node->spin();

  rclcpp::shutdown();
  return 0;
}
