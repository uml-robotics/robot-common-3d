#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "pcl_filter_3d_msgs/srv/pcl_voxel_grid_filter.hpp"

using pcl_filter_3d_msgs::srv::PCLVoxelGridFilter;
const std::string g_PARAM_NAMESPACE{"filters.voxel_grid."};

class VoxelGridFilterService : public rclcpp::Node
{
protected:
  rclcpp::Service<PCLVoxelGridFilter>::SharedPtr voxel_grid_filter_service_;
  float leaf_size_x_;
  float leaf_size_y_;
  float leaf_size_z_;

public:
  /**
   * Class Constructor.
   *
   * Constructor for VoxelGridFilterService class.
   */
  VoxelGridFilterService()
  : rclcpp::Node("voxel_grid_filter_service")
  {

    // declare all parameters above service to avoid race condition with the service
    // being created before the paramters are declared
    leaf_size_x_ = declare_parameter<float>(g_PARAM_NAMESPACE + "leaf_size_x");
    leaf_size_y_ = declare_parameter<float>(g_PARAM_NAMESPACE + "leaf_size_y");
    leaf_size_z_ = declare_parameter<float>(g_PARAM_NAMESPACE + "leaf_size_z");

    // bind callback and create service
    auto callback = std::bind(
      &VoxelGridFilterService::voxel_grid_filter, this, std::placeholders::_1,
      std::placeholders::_2);

    voxel_grid_filter_service_ = create_service<PCLVoxelGridFilter>("voxelgrid_filter", callback);
  }

  /**
   * Downsample a PointCloud2 message by applying a voxelgrid filter.
   *
   * Given a PointCloud2 message, apply a voxelgrid filter and provide the
   * resulting PointCloud2 message. More information about pcl filters at:
   * https://pcl.readthedocs.io/projects/tutorials/en/master/# This filter:
   * https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html#voxelgrid
   *
   * @param[in] req sensor_msgs/msg/PointCloud2 A PointCloud2 message.
   * @param[out] res sensor_msgs/msg/PointCloud2 A PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool voxel_grid_filter(
    const PCLVoxelGridFilter::Request::SharedPtr req, PCLVoxelGridFilter::Response::SharedPtr res)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    fromROSMsg(req->cloud_in, *input_cloud);

    get_parameter(g_PARAM_NAMESPACE + "leaf_size_x", leaf_size_x_);
    get_parameter(g_PARAM_NAMESPACE + "leaf_size_y", leaf_size_y_);
    get_parameter(g_PARAM_NAMESPACE + "leaf_size_z", leaf_size_z_);

    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(input_cloud);
    vox.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);
    vox.filter(*filtered_cloud);

    toROSMsg(*filtered_cloud, res->cloud_out);

    return true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelGridFilterService>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
