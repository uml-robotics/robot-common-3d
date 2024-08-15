#include <functional>  // std::bind
#include <memory>  // std::make_shared
#include <string_view>  // std::string_view
#include <utility>  // std::move

#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "armada_utility_msgs/srv/pcl_passthrough_filter.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud2.h"
#include "pcl/filters/passthrough.h"
#include "pcl_conversions/pcl_conversions.h"

using armada_utility_msgs::srv::PCLPassthroughFilter;
constexpr std::string_view g_PARAM_NAMESPACE = "filters.passthrough.";

class PassthroughFilterService : public rclcpp::Node {
private:
  rclcpp::Service<PCLPassthroughFilter>::SharedPtr passthrough_filter_service_;

  float x_min_;
  float x_max_;
  float y_min_;
  float y_max_;
  float z_min_;
  float z_max_;

public:
  /**
   * Class Constructor.
   *
   * Constructor for PassthroughFilterService class.
   */
  PassthroughFilterService()
  : rclcpp::Node("passthrough_filter_service")
  {
    declare_parameter<float>(g_PARAM_NAMESPACE + "x.min");
    declare_parameter<float>(g_PARAM_NAMESPACE + "x.max");
    declare_parameter<float>(g_PARAM_NAMESPACE + "y.min");
    declare_parameter<float>(g_PARAM_NAMESPACE + "y.max");
    declare_parameter<float>(g_PARAM_NAMESPACE + "z.min");
    declare_parameter<float>(g_PARAM_NAMESPACE + "z.max");

    auto callback =
        std::bind(&PassthroughFilterService::passthrough_filter, this,
                  std::placeholders::_1, std::placeholders::_2);

    passthrough_filter_service_ =
      create_service<PCLPassthroughFilter>
      ("passthrough_filter", std::move(callback));
  }

  /**
   * Apply a passthrough (x,y,z) filter to a PointCloud2 message.
   *
   * Given a PointCloud2 message, apply a passthrough (x,y,z) filter and
   * provide the resulting PointCloud2 message. More information about pcl filters at:
   * https://pcl.readthedocs.io/projects/tutorials/en/master/# This filter:
   * https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html#passthrough
   *
   * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
   * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool passthrough_filter(
    PCLPassthroughFilter::Request::SharedPtr req,
    PCLPassthroughFilter::Response::SharedPtr res)
  {
    get_parameter(g_PARAM_NAMESPACE + "x.min", x_min_);
    get_parameter(g_PARAM_NAMESPACE + "x.max", x_max_);
    get_parameter(g_PARAM_NAMESPACE + "y.min", y_min_);
    get_parameter(g_PARAM_NAMESPACE + "y.max", y_max_);
    get_parameter(g_PARAM_NAMESPACE + "z.min", z_min_);
    get_parameter(g_PARAM_NAMESPACE + "z.max", z_max_);

    pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
    moveFromROSMsg(req->cloud_in, temp_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    pass_x.setInputCloud(temp_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min_, x_max_);
    // pass_x.setFilterLimitsNegative(false);
    pass_x.filter(temp_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    pass_y.setInputCloud(temp_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min_, y_max_);
    pass_y.filter(temp_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud(temp_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min_, z_max_);
    pass_z.filter(temp_cloud);

    toROSMsg(temp_cloud, res->cloud_out);
    return true;
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PassthroughFilterService>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
