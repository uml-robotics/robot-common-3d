/*
 * Author: Brian Flynn
 * Date: Nov 15, 2022
 * Editors: Christian Tagliamonte
 * Last Modified: Jan 2, 2025
 * Adapted from:
 * https://github.com/uml-robotics/armada_behaviors/blob/main/armada_flexbe_utilities/src/service/pcl_concatenate_pointcloud_service.cpp
 *
 * Description: Starts up a service for extracting point clusters
 *   from an input point cloud message. Thus script returns a list of
 *   individual point cloud messages for each identified cluster.
 *
 * Input: sensor_msgs/PointCloud2
 * Output: sensor_msgs/PointCloud2[]
 *
 * Usage:
 *    `ros2 launch pcl_utilities concatenate_point_cloud.xml`
 */
#include <cassert>  // assert
#include <cstdint>  // uintmax_t
#include <functional>  // for std::bind
#include <limits>  // std::numeric_limits
#include <memory>  // std::make_shared
#include <string_view>  // std::string_view
#include <utility>  // std::move
#include <vector>  // std::string

#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl_conversions/pcl_conversions.h"

#include "pcl_utility_msgs/srv/pcl_euclidean_cluster_extraction.hpp"

using pcl_utility_msgs::srv::PCLEuclideanClusterExtraction;
constexpr std::string_view kParamNamespaceView = "filters.euclidean_cluster_extraction.";

namespace
{
class EuclideanClusterExtractionService : public rclcpp::Node
{
private:
  rclcpp::Service<PCLEuclideanClusterExtraction>::SharedPtr
    euclidean_cluster_extraction_service_;
  rcl_interfaces::msg::ParameterDescriptor param_constraints_;
  double cluster_tolerance_;
  int64_t min_cluster_size_;
  int64_t max_cluster_size_;

public:
  /**
   * Class Constructor.
   *
   * Constructor for EuclideanClusterExtractionService class.
   */
  EuclideanClusterExtractionService()
  : rclcpp::Node("euclidean_cluster_extraction_service")
  {
    const std::string kParamNamespace {kParamNamespaceView};

    // cap the parameter value within the range of pcl::uindex_t
    rcl_interfaces::msg::IntegerRange integer_range;
    auto value_or_max = std::min<uintmax_t>(
      static_cast<uintmax_t>(std::numeric_limits<int64_t>::max()),
      std::numeric_limits<pcl::uindex_t>::max());

    integer_range.from_value = 0;
    integer_range.to_value = static_cast<int64_t>(value_or_max);
    param_constraints_.integer_range.push_back(integer_range);

    // declare all parameters
    cluster_tolerance_ = declare_parameter<double>(
      kParamNamespace + "cluster_tolerance");

    min_cluster_size_ = declare_parameter<int64_t>(
      kParamNamespace + "min_cluster_size", param_constraints_);
    max_cluster_size_ = declare_parameter<int64_t>(
      kParamNamespace + "max_cluster_size", param_constraints_);

    // create callback and setup service
    auto callback = std::bind(
      &EuclideanClusterExtractionService::euclidean_cluster_extraction,
      this, std::placeholders::_1, std::placeholders::_2);

    euclidean_cluster_extraction_service_ = create_service<PCLEuclideanClusterExtraction>(
      "euclidean_cluster_extraction", std::move(callback));
  }

  /**
   * Segment clusters within a PointCloud into individual cloud objects.
   *
   * Given a PointCloud2 message, segment clusters of points into their
   * own PointCloud2 objects for further processing/handling.
   *
   * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
   * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool euclidean_cluster_extraction(
    PCLEuclideanClusterExtraction::Request::SharedPtr req,
    PCLEuclideanClusterExtraction::Response::SharedPtr res)
  {
    const std::string kParamNamespace {kParamNamespaceView};

    get_parameter(kParamNamespace + "cluster_tolerance", cluster_tolerance_);
    get_parameter(kParamNamespace + "min_cluster_size", min_cluster_size_);
    get_parameter(kParamNamespace + "max_cluster_size", max_cluster_size_);

    auto input_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    sensor_msgs::msg::PointCloud2 temp_cloud;
    std::vector<sensor_msgs::msg::PointCloud2> obstacle_cloud_list_out;

    pcl::moveFromROSMsg(req->cloud_in, *input_cloud);

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    tree->setInputCloud(input_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    std::vector<pcl::PointIndices> cluster_indices;

    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(static_cast<pcl::uindex_t>(min_cluster_size_));
    ec.setMaxClusterSize(static_cast<pcl::uindex_t>(max_cluster_size_));
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB> cloud_cluster;

    for (const pcl::PointIndices & indecies : cluster_indices) {
      cloud_cluster.clear();

      for (pcl::index_t index : indecies.indices) {
        // This should always be true, this will become relevent
        // if PCL extends the underlying pcl::index_t type to 64 bits
        assert(
          index >= 0 &&
          static_cast<uintmax_t>(index) <= uintmax_t{std::numeric_limits<size_t>::max()});

        cloud_cluster.push_back((*input_cloud)[static_cast<size_t>(index)]);
      }

      assert(
        uintmax_t{cloud_cluster.size()} <=
        uintmax_t{std::numeric_limits<pcl::uindex_t>::max()});

      cloud_cluster.width = static_cast<pcl::uindex_t>(cloud_cluster.size());
      cloud_cluster.height = 1U;
      cloud_cluster.is_dense = true;
      cloud_cluster.header.frame_id = input_cloud->header.frame_id;

      pcl::toROSMsg(cloud_cluster, temp_cloud);
      res->cloud_list_out.push_back(std::move(temp_cloud));
    }

    return true;
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EuclideanClusterExtractionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
