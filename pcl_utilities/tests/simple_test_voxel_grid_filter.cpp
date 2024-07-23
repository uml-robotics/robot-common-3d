#include <string>
#include <functional> // std::bind, std::placeholders
#include <future>
#include <memory> // std::shared_ptr, std::make_shared
#include <sstream> // std::stringstream
#include <ios> // std::fixed, std::set_percision

#include <rclcpp/node.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_utility_msgs/srv/pcl_voxel_grid_filter.hpp>

using pcl_utility_msgs::srv::PCLVoxelGridFilter;
using sensor_msgs::msg::PointCloud2;

class TestVoxelFilterServiceNode: public rclcpp::Node
{
private:
    rclcpp::Client<PCLVoxelGridFilter>::SharedPtr voxel_grid_filter_client_;
    rclcpp::Subscription<PointCloud2>::SharedPtr camera_subscription_;
    rclcpp::Publisher<PointCloud2>::SharedPtr output_publisher_;
    rclcpp::SubscriptionOptions subscriber_options_;

public:
  TestVoxelFilterServiceNode(): rclcpp::Node("voxel_filter_service_visualizer_node")
  {
    std::string camera_topic = declare_parameter<std::string>("point_cloud_topic");
    std::string client_topic = declare_parameter<std::string>("node_client_name");

    voxel_grid_filter_client_ = create_client<PCLVoxelGridFilter>(client_topic);

    output_publisher_ = create_publisher<PointCloud2>(
      "voxel_grid_filter/cloud_filtered", 1);

    // establish callback last to avoid race conditions

    // ros2 implicitely calls a callback when retrieving the result
    // so the subscriber needs to be in a Re-entrant callback group
    // and the executor needs to be multthreaded. Otherwise
    // `client.async_send_request.get()` deadlocks
    subscriber_options_ = rclcpp::SubscriptionOptions();
    subscriber_options_.callback_group = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    auto camera_callback = std::bind(
      &TestVoxelFilterServiceNode::camera_subscription_callback,
      this, std::placeholders::_1);

    camera_subscription_ = create_subscription<PointCloud2>(
      camera_topic, 1, camera_callback, subscriber_options_);
  }

  void camera_subscription_callback(PointCloud2::SharedPtr point_cloud) {
    std::stringstream output_stream;
    output_stream << std::fixed << std::setprecision(4);

    // call service with pointcloud input
    auto request = std::make_shared<PCLVoxelGridFilter::Request>();
    request->cloud_in = std::move(*point_cloud);

    auto response_future =
      voxel_grid_filter_client_->async_send_request(request);

    // retreve time to preserve appromate time message was sent
    // to format any test failures.
    double current_time = get_clock()->now().seconds();

    PCLVoxelGridFilter::Response::SharedPtr response {response_future.get()};
    output_publisher_->publish(response->cloud_out);

    // The folling tests are stand-ins for future unit tests
    // ASSERTIONS:
    //    - if the input cloud has points, then the output cloud must have
    //      at least one
    //    - the size of cloud_out must be less than the size of the
    //      cloud_in.

    if (
      response->cloud_out.data.size() == 0 &&
      request->cloud_in.data.size() > 0)
    {
      output_stream << "Sample at time: " << current_time
        << "produced output with 0 size with nonzero sized input.";

      RCLCPP_ERROR(get_logger(), output_stream.str().c_str());
      output_stream.str(""); // clear the stream
    }

    if (request->cloud_in.data.size() < response->cloud_out.data.size())
    {
      output_stream << "Sample at time: " << current_time
        << "produces a pointcloud with more points than the input.";

      RCLCPP_ERROR(get_logger(), output_stream.str().c_str());
      output_stream.str("");
    }
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<TestVoxelFilterServiceNode>();
  executor.add_node(node);

  executor.spin();
  rclcpp::shutdown();
}