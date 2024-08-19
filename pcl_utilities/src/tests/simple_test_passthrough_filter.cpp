/*
 * Author: Christian Tagliamonte
 * Date: Aug 15, 2024
 * Editors: N/A
 * Last Modified: Aug 15, 2024
 *
 * Description: A node to test the passthrough_filter_service.
 * This node listens to point cloud data from a camera
 *   messages from the topic specified by the parameter, `point_cloud_topic.`
 * The point cloud is passed to the service, where all points outside of a bounding box
 * are removed. The bounds for the passthrough filter are controlled via ros parameters,
 * these parameters default to the shape of 1x1x1 meter unit-cube centered at (0, 0, 0).
 *
 * NOTES:
 * - That all of these parameters must produce a volume with a positive, non-zero size.
 * - None of the paramters can be NaN.
 * The node will crash if invalid input is passed to these parameters.
 *
 * The filtered point cloud is then published to the topic
 *   `test_passthrough_filter_service/passthrough_filtered`
 *
 * Usage:
 *    `ros2 launch pcl_utilities test_passthrough_filter_service.xml point_cloud_topic:=<POINT_CLOUD_TOPIC>`
 */
#include <chrono>   // std::chrono::seconds
#include <cstddef>  // size_t
#include <cstdint>  // int64_t
#include <functional>  // std::bind, std::placeholders
#include <ios>  // std::fixed, std::setprecision
#include <memory>  // std::make_shared
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

#include "pcl_utility_msgs/srv/pcl_passthrough_filter.hpp"

using pcl_utility_msgs::srv::PCLPassthroughFilter;
using sensor_msgs::msg::PointCloud2;

constexpr std::chrono::seconds MAX_WAIT_TIME {1U};

class TestPassthroughFilterNode : public rclcpp::Node
{
private:
  rclcpp::Client<PCLPassthroughFilter>::SharedPtr
    passthrough_filter_client_;
  rclcpp::Publisher<PointCloud2>::SharedPtr output_publisher_;
  std::string camera_topic_;

public:
  TestPassthroughFilterNode()
  : rclcpp::Node("simple_test_passthrough_filter")
  {
    std::string client_topic = declare_parameter<std::string>("node_client_name");
    camera_topic_ = declare_parameter<std::string>("point_cloud_topic");

    passthrough_filter_client_ =
      create_client<PCLPassthroughFilter>(client_topic);

    output_publisher_ = create_publisher<PointCloud2>(
      "test_passthrough_filter_service/cloud_filtered", 1);
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
    auto request = std::make_shared<PCLPassthroughFilter::Request>();
    request->cloud_in = point_cloud;

    auto response_future =
      passthrough_filter_client_->async_send_request(request);

    auto response_code = rclcpp::spin_until_future_complete(
      shared_from_this(), response_future, MAX_WAIT_TIME);

    if (response_code != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to recieve a response from the service");
      return;
    }

    PCLPassthroughFilter::Response::SharedPtr response {
      response_future.get()};
    output_publisher_->publish(response->cloud_out);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestPassthroughFilterNode>();
  node->spin();

  rclcpp::shutdown();
  return 0;
}
