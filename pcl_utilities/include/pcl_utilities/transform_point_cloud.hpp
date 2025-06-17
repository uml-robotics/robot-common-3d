#ifndef ROBOT_COMMON_3D_SRC_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_POINT_CLOUD_TRANSFORMER_H_
#define ROBOT_COMMON_3D_SRC_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_POINT_CLOUD_TRANSFORMER_H_
#include "rclcpp/clock.hpp"  // rclcpp::Clock
#include "rclcpp/visibility_control.hpp"  // RCLCPP_EXPORT

#include "tf2_ros/buffer.h" // tf2_ros::Buffer
#include "tf2_ros/transform_listener.h"  // tf2_ros::TransformListener

#include "pcl_utility_msgs/srv/pcl_transform_point_cloud.hpp"  // pcl_utility_msgs::srv::PCLTransformPointCloud

namespace pcl_utilities
{
using pcl_utility_msgs::srv::PCLTransformPointCloud;

struct PointCloudTransformer
{
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  RCLCPP_EXPORT explicit PointCloudTransformer(rclcpp::Clock::SharedPtr clock)
  : buffer_{clock}, listener_{buffer_}
  {
  }

  /**
 * @brief Transform an array of PointCloud2 into a single PointCloud2.
 *
 * @details Given an array of PointCloud2 messages, transform into a single PointCloud2
 *  message. If the transform cannot be found within req.max_transform_attempts, then the
 *  this function will throw.
 *
 * @param[in] req sensor_msgs/PointCloud2 Container of PointCloud2 messages.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 * @throw std::runtime_error If the transform cannot be found in `req.max_transform_attempts`
 */
  RCLCPP_EXPORT void operator()(
    const PCLTransformPointCloud::Request & req, PCLTransformPointCloud::Response & res) const;
};
}  // namespace pcl_utilities
#endif
