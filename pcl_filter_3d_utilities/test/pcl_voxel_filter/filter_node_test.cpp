#include <string>
#include <functional>

#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pcl_filter_3d_utilities/srv/pcl_voxel_grid_filter.hpp"

using pcl_filter_3d_msgs::srv::PCLVoxelGridFilter;
using sensor_msgs::msg::PointCloud2;
const std::string g_PARAM_NAMESPACE{"filters.voxel_grid."};

class VoxelFilterServiceTestNode: public rclcpp::Node
{
private:
    std::string camera_topic_;
    rclcpp::Client<PCLVoxelGridFilter>::SharedPtr voxelgrid_filter_client_;
    rclcpp::Subscription<PointCloud2>::SharedPtr camera_subscription_;
    rclcpp::Publisher<PointCloud2>::SharedPtr rviz_output_publisher_;

public:
  VoxelFilterServiceTestNode(): rclcpp::Node("voxel_filter_service_test_node")
  {
    camera_topic_ = declare_parameter<std::string>("camera_topic_");

    voxelgrid_filter_client_ = create_client<PCLVoxelGridFilter>(
        "voxelgrid_filter");

    rviz_output_publisher_ = create_publisher<PointCloud2>("voxel_filter_output");

    auto callback =
  }

  void camera_subscription_callback(PointCloud2::ConstSharedPtr point_cloud) {
    
    
    voxelgrid_filter_client_->async_send_request()
  }


}