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

#include <pcl/point_cloud.h>  // pcl::PointCloud
#include <pcl_conversions/pcl_conversions.h>  // std::shared_ptr

#include <cstddef>  // size_t
#include <memory>  // std::make_shared
#include <utility>  // std::move

#include <pcl/impl/point_types.hpp>
#include <pcl_utility_msgs/srv/pcl_concatenate_point_cloud.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using pcl_utility_msgs::srv::PCLConcatenatePointCloud;
constexpr size_t MAX_POINT_CLOUDS = 400U;

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
    const rclcpp::Logger & node_logger,
    PCLConcatenatePointCloud::Request::SharedPtr req,
    PCLConcatenatePointCloud::Response::SharedPtr res) {
  unsigned int cloud_list_size = static_cast<unsigned int>(req->cloud_list_in.size());

  static_assert(sizeof(pcl::PointCloud<pcl::PointXYZRGB>) != 0);

  // make sure the number of point clouds can safely
  // fit in stack memory
  if (cloud_list_size > MAX_POINT_CLOUDS) {
    RCLCPP_ERROR_STREAM(
        node_logger,
        "the number of pointclouds must be less than " << MAX_POINT_CLOUDS);
    return false;
  }

  // header.frame_id must be managed separately since PCL::PointCloud
  // does not track frame_id
  std::string frame_id;
  for (unsigned int i = 0; i < cloud_list_size; i++) {
    const std::string current_frame_id = req->cloud_list_in[i].header.frame_id;

    if (frame_id == "") {
      frame_id = current_frame_id;
    }

    if (frame_id != current_frame_id) {
      // header.frame_id is not tracked in PCL::PointCloud, must
      // be managed separetely by ROS for concatenation
      RCLCPP_ERROR_STREAM(node_logger,
        "the pointcloud at index #" << i << "with the frame of "
        << std::quoted(current_frame_id) << "does not match the "
        << "required frame id of" << std::quoted(frame_id));
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud_array[cloud_list_size];
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenated_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  for (unsigned int i = 0; i < cloud_list_size; ++i) {
    fromROSMsg(req->cloud_list_in[i], input_cloud);
    cloud_array[i] = input_cloud;
  }

  for (unsigned int i = 0; i < cloud_list_size; i++) {
    // PointCloud::operator+= manages is_dense and timestamp feilds
    // internally
    *concatenated_cloud += cloud_array[i];
  }

  toROSMsg(*concatenated_cloud, res->cloud_out);

  res->cloud_out.header.frame_id = std::move(frame_id);
  return true;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("concatenate_point_cloud_service");
  auto callback = std::bind(
    concatenate_point_cloud, node->get_logger(),
    std::placeholders::_1, std::placeholders::_2);

  auto dummy_subscriber = node->create_service<PCLConcatenatePointCloud>(
      "concatenate_point_cloud", std::move(callback));
  (void)dummy_subscriber;

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}