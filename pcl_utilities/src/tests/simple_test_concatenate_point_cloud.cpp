/*
 * Author: Christian Tagliamonte
 * Date: Aug Aug 4, 2024
 * Editors: N/A
 * Last Modified: Aug 7, 2024
 *
 * Description: A node to test the concatenate_point_cloud_service.
 * This node listens to a series of sequential pointcloud
 *   messages from the topic specified by the parameter, `point_cloud_topic.`
 * Each message is stored within a queue with a
 *   size specified by the parameter, `num_point_clouds` (defaults to 5). When
 *   the queue is full, all messages in are sent
 *   to the `concatenate_point_cloud_service`.
 *
 * The concatenated point cloud is then published to the topic
 *   `concatenate_point_cloud/cloud_concatenated`
 *
 * Usage:
 *    `ros2 launch pcl_utilities test_concatenate_point_cloud.xml point_cloud_topic:=<POINT_CLOUD_TOPIC>`
 */
#include <chrono>   // std::chrono::seconds
#include <cstddef>  // size_t
#include <cstdint>  // int64_t
#include <deque>
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
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_utility_msgs/srv/pcl_concatenate_point_cloud.hpp"

using pcl_utility_msgs::srv::PCLConcatenatePointCloud;
using sensor_msgs::msg::PointCloud2;

constexpr size_t MAX_POINT_CLOUDS = 100U;
constexpr std::chrono::seconds MAX_WAIT_TIME {1U};

class TestConcatenatePointCloudNode: public rclcpp::Node
{
private:
    rclcpp::Client<PCLConcatenatePointCloud>::SharedPtr
      concatenate_point_cloud_client_;
    rclcpp::Publisher<PointCloud2>::SharedPtr output_publisher_;
    std::deque<PointCloud2> history_;
    std::string camera_topic_;
    size_t num_point_clouds_;

public:
  TestConcatenatePointCloudNode()
  : rclcpp::Node("simple_test_concatenate_point_cloud")
  {
    std::string client_topic = declare_parameter<std::string>("node_client_name");
    camera_topic_ = declare_parameter<std::string>("point_cloud_topic");

    // rclcpp uses int64_t internally, use explicit cast to size_t
    rcl_interfaces::msg::ParameterDescriptor param_constraints;
    rcl_interfaces::msg::IntegerRange integer_range;
    integer_range.from_value = 0U;
    integer_range.to_value = MAX_POINT_CLOUDS;
    param_constraints.integer_range.push_back(integer_range);

    num_point_clouds_ = static_cast<size_t>(
      declare_parameter<int64_t>("num_point_clouds", std::move(param_constraints)));

    concatenate_point_cloud_client_ =
      create_client<PCLConcatenatePointCloud>(client_topic);

    output_publisher_ = create_publisher<PointCloud2>(
      "concatenate_point_cloud/cloud_concatenated", 1);

    history_ = std::deque<PointCloud2>();
  }

  void spin()
  {
    while(rclcpp::ok())
    {
      PointCloud2 point_cloud_message;
      bool was_retrieved = rclcpp::wait_for_message(
        point_cloud_message, shared_from_this(),
        camera_topic_, MAX_WAIT_TIME);

      if (!was_retrieved)
      {
        RCLCPP_ERROR(get_logger(),
          "A camera message could not be retrieved within 1 second.");
        continue;
      }

      process_point_cloud(point_cloud_message);
    }
  }

  void process_point_cloud(PointCloud2& point_cloud) {
    // create a fixed sized context window of old point clouds
    history_.push_back(std::move(point_cloud));
    if (history_.size() > num_point_clouds_)
    {
      history_.pop_front();
    } else if (history_.size() < num_point_clouds_)
    {
      return;
    }

    auto request = std::make_shared<PCLConcatenatePointCloud::Request>();
    request->cloud_list_in = std::vector(history_.cbegin(), history_.cend());

    auto response_future =
      concatenate_point_cloud_client_->async_send_request(request);

    auto response_code = rclcpp::spin_until_future_complete(
      shared_from_this(), response_future, MAX_WAIT_TIME);

    if (response_code != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to recieve a response from the service");
      return;
    }

    PCLConcatenatePointCloud::Response::SharedPtr response {
      response_future.get()};
    output_publisher_->publish(response->cloud_out);

    test_point_cloud(request, response);
  }

  void test_point_cloud(
    PCLConcatenatePointCloud::Request::SharedPtr,
    PCLConcatenatePointCloud::Response::SharedPtr response)
  {
    std::stringstream output_stream;
    output_stream << std::fixed << std::setprecision(2);

    // PCL takes the maximum timestamp when concatenating messages
    // The time stamp of the newest message passed to the service.
    double current_time = rclcpp::Time(response->cloud_out.header.stamp).seconds();

    // The folling tests are stand-ins for future unit tests
    // ASSERTIONS:
    //  - The ouput point cloud must be at least the size of the
    //    largest input cloud.
    std::size_t max_point_cloud_size = 0U;
    for (const auto& point_cloud : history_) {
        max_point_cloud_size = std::max(max_point_cloud_size, point_cloud.data.size());
    }

    if (
      response->cloud_out.data.size() < max_point_cloud_size)
    {
      output_stream << "Sample at time: " << current_time
        << "produced an output point cloud with a size "
        << "less than an input point cloud.";

      RCLCPP_ERROR(get_logger(), output_stream.str().c_str());
      output_stream.str(""); // clear the stream
    }
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestConcatenatePointCloudNode>();
  node->spin();

  rclcpp::shutdown();
}