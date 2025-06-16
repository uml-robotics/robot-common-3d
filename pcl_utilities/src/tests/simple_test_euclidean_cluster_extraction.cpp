/*
 * Author: Christian Tagliamonte
 * Date: Aug 4, 2024
 * Editors: N/A
 * Last Modified: Jan 2, 2025
 *
 * Description:
 *    This script tests the euclidean_cluster extraction service by providng
 *    a series of downsampled point cloud messages from a camera to the service
 *    and publishing the result to a topic. See the full description below.
 *
 * Steps:
 *   This script performs the following procedures in order:
 *   1) Listen for a point cloud message from the topic specified by
 *      the ROS parameter, `point_cloud_topic.`
 *   2) Crop (pcl::CropBox) then downsample (pcl::VoxelGridFilter) the
 *      point cloud to speed up the clustering algorithm.
 *   3) Call the `euclidean_cluster_extraction_service` node with the
 *      the point cloud. This service returns a list of clusters as
 *      individual point clouds.
 *   4) Give each group visually idenfifiable tint and concatenate
 *      all groups to produce a final point cloud.
 *   5) Publish the final point cloud to the topic,
 *      `euclidan_cluster_extraction/cloud_concatenated`
 *
 * Usage:
 *    `ros2 launch pcl_utilities test_euclidan_cluster_extraction.xml
 * point_cloud_topic:=<POINT_CLOUD_TOPIC>`
 */

#include <algorithm>  // std::min
#include <chrono>  // std::chrono::seconds
#include <cstddef>  // size_t
#include <cstdint>  // uint8_t
#include <memory>  // std::make_shared
#include <string>   // std::string
#include <utility>  // std::move

#include "rclcpp/executors.hpp"  // rclcpp::spin_until_future_complete
#include "rclcpp/logging.hpp"  // RCLCPP_ERROR_STREAM
#include "rclcpp/node.hpp"  // rclcpp::node
#include "rclcpp/publisher.hpp"  // rclcpp::Publisher<SrvT>
#include "rclcpp/utilities.hpp"  // rclcpp::init, rclcpp::shutdown
#include "rclcpp/wait_for_message.hpp"  // rclcpp::wait_for_message<MsgT>

#include "Eigen/Core"  // Eigen::Vector4f

#include "pcl_conversions/pcl_conversions.h"  // fromROSMsg, toROSMsg

#include "pcl/filters/crop_box.h" // pcl::CropBox<PointT>
#include "pcl/filters/voxel_grid.h" // pcl::VoxelGrid<PointT>
#include "pcl/memory.h"  // pcl::make_shared<T, Args...>
#include "pcl/point_cloud.h"  // pcl::PointCloud<PointT>
#include "pcl/point_types.h"  // pcl::PointXYZRGB, pcl::PointXYZHSV
#include "pcl/point_types_conversion.h"  // pcl::PointXYZHSVtoXYZRGB

#include "pcl_utility_msgs/srv/pcl_euclidean_cluster_extraction.hpp" // pcl_utility_msgs::srv::PCLEucideanClusterExtraction

#include "sensor_msgs/msg/point_cloud2.hpp"  // sensor_msgs::msg::PointCloud

#include "pcl_utilities/detail/numeric_utils.hpp"  // pcl_utilities::detail::narrowing_cast

using pcl_utility_msgs::srv::PCLEuclideanClusterExtraction;
using ROSPointCloud2 = sensor_msgs::msg::PointCloud2;
using pcl_utilities::detail::narrowing_cast;

namespace
{
constexpr std::chrono::seconds kMaxWaitTime{5U};
float lerp_float(float first, float second, float factor);

class TestEuclidanClusterExtractionNode : public rclcpp::Node
{
private:
  rclcpp::Client<PCLEuclideanClusterExtraction>::SharedPtr euclidan_cluster_extraction_client_;
  rclcpp::Publisher<ROSPointCloud2>::SharedPtr output_publisher_;
  std::string camera_topic_;

public:
  TestEuclidanClusterExtractionNode()
  : rclcpp::Node("simple_test_euclidan_cluster_extraction")
  {
    std::string client_topic = declare_parameter<std::string>("node_client_name");
    camera_topic_ = declare_parameter<std::string>("point_cloud_topic");

    euclidan_cluster_extraction_client_ =
      create_client<PCLEuclideanClusterExtraction>(client_topic);

    output_publisher_ = create_publisher<ROSPointCloud2>("euclidan_cluster_extraction/clusters", 1);
  }

  void spin()
  {
    ROSPointCloud2 point_cloud_message;

    while (rclcpp::ok()) {
      bool was_retrieved = rclcpp::wait_for_message(
        point_cloud_message, shared_from_this(), camera_topic_, kMaxWaitTime);

      if (!was_retrieved) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "A camera message could not be retrieved within 1 second.");
        continue;
      }

      process_point_cloud(std::move(point_cloud_message));
    }
  }

  /**
   * This processes the point cloud by downsampling it, calling the clustering
   * service, concatenating/colorizing all clustered segments into a final
   * point cloud, then publishes it.
   *
   * @param point_cloud sensor_msgs/PointCloud2 message to be processed.
   */
  void process_point_cloud(ROSPointCloud2 point_cloud)
  {
    // Do this to reduce the number of total points to speed up computation
    crop_downsample_point_cloud(point_cloud);

    if (point_cloud.data.empty()) {
      return;
    }

    auto request = std::make_shared<PCLEuclideanClusterExtraction::Request>();
    request->cloud_in = std::move(point_cloud);

    auto response_future = euclidan_cluster_extraction_client_->async_send_request(request);

    auto response_code =
      rclcpp::spin_until_future_complete(shared_from_this(), response_future, kMaxWaitTime);

    if (response_code != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to recieve a response from the service");
      return;
    }

    std::shared_ptr response{response_future.get()};
    output_publisher_->publish(
      colorize_concatenate_point_clouds(
        std::move(response->cloud_list_out), std::move(request->cloud_in.header.frame_id)));
  }

  /**
   * Reduce the number of points in a pointcloud by performing a box crop and
   * voxel grid filter. The resulting point cloud overwrites the original.
   *
   *
   * @param point_cloud sensor_msgs/PointCloud2 message to be filtered.
   */
  void crop_downsample_point_cloud(ROSPointCloud2 & point_cloud)
  {
    // Create a box of 0.5m x 0.5m x 1.0m. Depth of 1m.
    // Downsample the point cloud by aggregating voxels for every point within
    // 0.25mm. Note that these variables must not be given a static lifetime
    // since Eigen::Vector<N>f is not guarenteed to be trivially constructable
    // by constexpr
    const Eigen::Vector4f kBoxCoordinatesMin{-0.25f, -0.25f, 0.f, 0.f};
    const Eigen::Vector4f kBoxCoordinatesMax{0.25f, 0.25f, 1.f, 0.f};
    const Eigen::Vector4f kVoxelGridLeafSize{0.003f, 0.003f, 0.003f, 0.f};

    if (point_cloud.data.empty()) {
      return;
    }

    auto pcl_point_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::moveFromROSMsg(point_cloud, *pcl_point_cloud);

    // Crop the point cloud, more points further away from the camera are
    // removed This step has the greatest effect reducing points
    pcl::CropBox<pcl::PointXYZRGB> crop_box;
    crop_box.setMax(kBoxCoordinatesMax);
    crop_box.setMin(kBoxCoordinatesMin);
    crop_box.setInputCloud(pcl_point_cloud);
    crop_box.filter(*pcl_point_cloud);

    // if there are nonzero points, perform a voxel filter
    // to reduce the number of points passed to the clustering
    // step. This is good for reducing more points closer
    // to the camera
    if (!pcl_point_cloud->empty()) {
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
      voxel_grid.setInputCloud(pcl_point_cloud);
      voxel_grid.setLeafSize(kVoxelGridLeafSize);
      voxel_grid.filter(*pcl_point_cloud);
    }

    pcl::toROSMsg(*pcl_point_cloud, point_cloud);
  }

  /**
   * Given a list of point clouds, apply a distinct colored tint to
   * each point cloud, then concatenate all point clouds into one final
   * resulting point cloud.
   *
   * @param point_clouds sensor_msgs/PointCloud2[] message to be
   * colorized/concatenated.
   * @param frame_id the frame ID of the final point cloud
   */
  ROSPointCloud2 colorize_concatenate_point_clouds(
    std::vector<ROSPointCloud2> point_clouds, std::string frame_id)
  {
    constexpr float kTintColorWeight = 0.2f;  // 60 out of 255, weight in [0.0f, 1.0f]
    constexpr size_t kMaxNumClusters = 20U;   // 18 degree minimum hue-step out of 360 per cluster

    pcl::PointCloud<pcl::PointXYZRGB> pcl_temp_point_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_concatenated_point_cloud;

    // HSV order, H in [0,360.f], S in [0.f, 1.f], V in [0.f, 1.f]
    pcl::PointXYZHSV hsv_color_tint{0.0f, 1.0f, 1.0f};
    pcl::PointXYZRGB rgb_color_tint;

    size_t num_clusters = std::min(point_clouds.size(), kMaxNumClusters);

    for (size_t i = 0; i < num_clusters; i++) {
      // Adjust the hue of the point so there will be apparent differences in
      // color between the clusters
      hsv_color_tint.h = 360.0f * float(i) / float(num_clusters);

      pcl::PointXYZHSVtoXYZRGB(hsv_color_tint, rgb_color_tint);
      pcl::moveFromROSMsg(point_clouds[i], pcl_temp_point_cloud);

      for (pcl::PointXYZRGB & point : pcl_temp_point_cloud) {
        // Lerp the current point color with the tinted point
        // (r, g, b) <= 255.0 + epsilon. Where epsilon is truncated via the
        // proceeding cast.
        float r = lerp_float(float(rgb_color_tint.r), float(point.r), kTintColorWeight);
        float g = lerp_float(float(rgb_color_tint.g), float(point.g), kTintColorWeight);
        float b = lerp_float(float(rgb_color_tint.b), float(point.b), kTintColorWeight);

        // The values above will fit within uint8_t
        // as long as (1) 0.0 <= COLOR_RANGE <= 1.0 and (2) the size of each
        // channel in PointRGBXYZ is at most 1 byte PCL uses overlapping union
        // members to define color, this is considered UB but there is no way
        // around it: see https://github.com/PointCloudLibrary/pcl/issues/2303
        point = pcl::PointXYZRGB(
          point.x, point.y, point.z, narrowing_cast<uint8_t>(r), narrowing_cast<uint8_t>(g),
          narrowing_cast<uint8_t>(b));
      }

      // concatenate the colorized point cluster into one point cloud
      pcl_concatenated_point_cloud += pcl_temp_point_cloud;
    }

    // convert back to a ROS message and return
    ROSPointCloud2 output_point_cloud;
    pcl::toROSMsg(pcl_concatenated_point_cloud, output_point_cloud);
    output_point_cloud.header.frame_id = std::move(frame_id);
    return output_point_cloud;
  }
};

float lerp_float(float first, float second, float factor)
{
  return factor * first + (1.0f - factor) * second;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestEuclidanClusterExtractionNode>();
  node->spin();

  rclcpp::shutdown();
  return 0;
}
