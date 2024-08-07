/*
 * Author: Christian Tagliamonte
 * Date: Aug Aug 4, 2024
 * Editors: N/A
 * Last Modified: Aug 4, 2024
 *
 * Description: A node to test the concatenate_point_cloud_service. This node listens to a series of sequential pointcloud
 * messages from the topic specified by the parameter, `point_cloud_topic.` Each message is stored within a queue with a
 * size specified by the parameter, `num_point_clouds` (defaults to 5). When the queue is full, all messages in are sent
 * to the `concatenate_point_cloud_service`.
 *
 * The concatenated point cloud is then published to the topic `concatenate_point_cloud_filter/cloud_concatenated`
 *
 * Usage:
 *    `ros2 launch pcl_utilities test_concatenate_point_cloud_filter.xml point_cloud_topic:=<POINT_CLOUD_TOPIC>`
 */
#include <algorithm>  // std::clamp
#include <climits>
#include <cstdint>  // size_t
#include <deque>
#include <functional>  // std::bind, std::placeholders
#include <future>
#include <ios>  // std::fixed, std::set_percision
#include <memory>  // std::shared_ptr, std::make_shared
#include <sstream>  // std::stringstream
#include <string>
#include <mutex>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <pcl_utility_msgs/srv/pcl_concatenate_point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using pcl_utility_msgs::srv::PCLConcatenatePointCloud;
using sensor_msgs::msg::PointCloud2;

class TestConcatenatePointCloudNode: public rclcpp::Node
{
private:
    rclcpp::Client<PCLConcatenatePointCloud>::SharedPtr concatenate_point_cloud_filter_client_;
    rclcpp::Publisher<PointCloud2>::SharedPtr output_publisher_;
    std::deque<PointCloud2> history_;
    std::size_t num_point_clouds_;
    std::string camera_topic_;

public:
  TestConcatenatePointCloudNode(): rclcpp::Node("simple_test_concatenate_point_cloud")
  {
    camera_topic_ = declare_parameter<std::string>("point_cloud_topic");
    std::string client_topic = declare_parameter<std::string>("node_client_name");

    int64_t num_messages = declare_parameter<int64_t>("num_point_clouds");
    num_messages = std::clamp(num_messages, int64_t{0}, int64_t{INT_MAX});
    num_point_clouds_ = static_cast<std::size_t>(num_messages);

    concatenate_point_cloud_filter_client_ = create_client<PCLConcatenatePointCloud>
      (client_topic, rmw_qos_profile_services_default);

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
        camera_topic_, std::chrono::seconds(1));

      if (!was_retrieved)
      {
        RCLCPP_ERROR(get_logger(),
          "A camera message could not be retrieved in 1 second.");
        continue;
      }

      point_cloud_callback(point_cloud_message);
    }
  }

  void point_cloud_callback(PointCloud2& point_cloud) {
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
      concatenate_point_cloud_filter_client_->async_send_request(request);

    rclcpp::spin_until_future_complete(
      shared_from_this(), response_future);

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
    //    - if the input cloud has points, then the output cloud must have
    //      at least one
    //    - the size of cloud_out must be less than the size of the
    //      cloud_in.
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