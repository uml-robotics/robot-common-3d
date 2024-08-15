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

#include <cstddef>  // size_t
#include <functional>  // std::bind
#include <iomanip>  // std::quoted
#include <memory>  // std::make_shared
#include <sstream>
#include <string>
#include <utility>  // std::move

#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"

#include "pcl_utility_msgs/srv/pcl_concatenate_point_cloud.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using pcl_utility_msgs::srv::PCLConcatenatePointCloud;

class ConcatenatePointCloudService : public rclcpp::Node
{
private:
  rclcpp::Service<PCLConcatenatePointCloud>::SharedPtr
    concatenate_pointcloud_service_;

public:
  ConcatenatePointCloudService()
  : rclcpp::Node("concatenate_point_cloud_service")
  {
    auto callback = std::bind(
      &ConcatenatePointCloudService::concatenate_point_cloud,
      this, std::placeholders::_1, std::placeholders::_2);

    concatenate_pointcloud_service_ = create_service
      <PCLConcatenatePointCloud>("concatenate_point_cloud", std::move(callback));
  }

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
    PCLConcatenatePointCloud::Request::SharedPtr req,
    PCLConcatenatePointCloud::Response::SharedPtr res)
  {
    // header.frame_id must be managed separately since PCL::PointCloud
    // does not track frame_id
    // Assert that all input_clouds have the same frame_id
    std::string frame_id;
    for (size_t i = 0; i < req->cloud_list_in.size(); ++i) {
      const std::string current_frame_id =
        req->cloud_list_in[i].header.frame_id;

      if (frame_id == "") {
        frame_id = current_frame_id;
      }

      if (frame_id != current_frame_id) {
        // NOTE: RCLCPP_* macros are threadsafe
        // header.frame_id is not tracked in PCL::PointCloud, must
        // be managed separetely by ROS for concatenation
        // NOTE: The linter only approves of the format below
        std::stringstream error_message;

        error_message << "The point cloud at index #" << i << " has a frame_id of "
                      << std::quoted(current_frame_id) << "which does not match the "
                      << "the required frame_id of " << std::quoted(frame_id);

        RCLCPP_ERROR_STREAM(get_logger(), error_message.str());

        return false;
      }
    }

    pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> concatenated_cloud;

    for (auto & point_cloud : req->cloud_list_in) {
      // req->cloud_list_in[i] becomes invalidated
      moveFromROSMsg(point_cloud, input_cloud);
      // PointCloud::operator+= manages is_dense and timestamp feilds
      concatenated_cloud += input_cloud;
    }

    // If there are no point clouds, the output point cloud
    // has a frame id of "" and a default constructed point cloud
    // No function to move from pcl::PointCloud -> PointCloud2
    toROSMsg(concatenated_cloud, res->cloud_out);
    res->cloud_out.header.frame_id = std::move(frame_id);

    return true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConcatenatePointCloudService>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
