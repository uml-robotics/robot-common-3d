#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cassert>
#include <memory>
#include <utility>

#include <pcl_utility_msgs/srv/pcl_concatenate_point_cloud.hpp>
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using pcl_utility_msgs::srv::PCLConcatenatePointCloud;
const size_t MAX_POINT_CLOUDS = 400U;

/**
 * Concatenate an array of PointCloud2 into a single PointCloud2.
 *
 * Given an array of PointCloud2 messages, concatenate into a single PointCloud2
 * message.
 *
 * @param[in] req sensor_msgs/PointCloud2[] Container of PointCloud2 messages.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 * @return Bool Service completion result.
 */
bool concatenate_point_cloud(
  const rclcpp::Logger& node_logger,
  PCLConcatenatePointCloud::Request::SharedPtr req,
  PCLConcatenatePointCloud::Response::SharedPtr res)
{
  size_t cloud_list_size = req->cloud_list_in.size();

  // make sure the number of point clouds can safely
  // fit in stack memory
  if (cloud_list_size > MAX_POINT_CLOUDS) {
    RCLCPP_ERROR_STREAM(node_logger,
      "the number of pointclouds must be less than " << MAX_POINT_CLOUDS);
    return false;
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud_array[cloud_list_size];
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenated_cloud(
    new pcl::PointCloud<pcl::PointXYZRGB>);

  for (size_t i = 0; i < cloud_list_size; ++i) {
    fromROSMsg(req->cloud_list_in[i], input_cloud);
    cloud_array[i] = input_cloud;
  }

  for (size_t i = 0; i < cloud_list_size; i++) {
    *concatenated_cloud += cloud_array[i];
  }

  toROSMsg(*concatenated_cloud, res->cloud_out);
  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("concatenate_point_cloud_service");
  auto callback = std::bind(
    concatenate_point_cloud, node->get_logger(),
    std::placeholders::_1, std::placeholders::_2
  );

  auto dummy_subscriber = node->create_service<PCLConcatenatePointCloud>(
    "concatenate_point_cloud", callback);

  (void)dummy_subscriber;

  rclcpp::shutdown();

  return 0;
}