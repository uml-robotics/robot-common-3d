/*
 * Author: Brian Flynn
 * Date: Nov 17, 2022
 * Editors: Christian Tagliamonte
 * Last Modified: Aug 19, 2024
 * Adapted from:
 * https://github.com/uml-robotics/armada_behaviors/commits/main/armada_flexbe_utilities/src/service/pcl_passthrough_filter_service.cpp
 *
 * Description: Starts up a service for filtering point cloud data with a bounding box.
 *
 * Input: sensor_msgs/PointCloud2
 * Output: sensor_msgs/PointCloud2
 *
 * Usage:
 *    `ros2 launch pcl_utilities passthrough_filter.xml`
 */
#include <functional>  // std::bind
#include <limits>  // std::numeric_limits
#include <memory>  // std::make_shared
#include <sstream>  // std::stringstream
#include <string_view>  // std::string_view
#include <utility>  // std::move

#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"
#include "pcl/filters/passthrough.h"
#include "pcl_conversions/pcl_conversions.h"

#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_utility_msgs/srv/pcl_passthrough_filter.hpp"

using pcl_utility_msgs::srv::PCLPassthroughFilter;
constexpr std::string_view g_PARAM_NAMESPACE = "filters.passthrough.";

class PassthroughFilterService : public rclcpp::Node
{
private:
  rclcpp::Service<PCLPassthroughFilter>::SharedPtr passthrough_filter_service_;
  rcl_interfaces::msg::ParameterDescriptor constraints_;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;

public:
  /**
   * Class Constructor.
   *
   * Constructor for PassthroughFilterService class.
   */
  PassthroughFilterService()
  : rclcpp::Node("passthrough_filter_service")
  {
    std::string param_namespace {g_PARAM_NAMESPACE};

    // All parameters will need to be converted to float
    // This conversion is undefined if the values within the double
    // cannot fit within a float. For systems without an Infinity,
    // std::infinity defaults to the highest representable value.
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = double {-std::numeric_limits<float>::infinity()};
    float_range.to_value = double {std::numeric_limits<float>::infinity()};
    constraints_.floating_point_range.push_back(float_range);

    x_min_ = declare_parameter<double>(param_namespace + "x.min", constraints_);
    x_max_ = declare_parameter<double>(param_namespace + "x.max", constraints_);
    y_min_ = declare_parameter<double>(param_namespace + "y.min", constraints_);
    y_max_ = declare_parameter<double>(param_namespace + "y.max", constraints_);
    z_min_ = declare_parameter<double>(param_namespace + "z.min", constraints_);
    z_max_ = declare_parameter<double>(param_namespace + "z.max", constraints_);

    // do not shutdown so all memory is cleaned up before exit
    check_for_valid_limits();

    auto callback = std::bind(
      &PassthroughFilterService::passthrough_filter, this,
      std::placeholders::_1, std::placeholders::_2);

    passthrough_filter_service_ = create_service<PCLPassthroughFilter>(
      "passthrough_filter", std::move(callback));
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
    // Retrieve and validate all parameters
    std::string param_namespace {g_PARAM_NAMESPACE};

    get_parameter(param_namespace + "x.min", x_min_);
    get_parameter(param_namespace + "x.max", x_max_);
    get_parameter(param_namespace + "y.min", y_min_);
    get_parameter(param_namespace + "y.max", y_max_);
    get_parameter(param_namespace + "z.min", z_min_);
    get_parameter(param_namespace + "z.max", z_max_);

    if (!check_for_valid_limits()) {
      rclcpp::shutdown();
      return false;
    }

    auto temp_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    std::string frame_id = req->cloud_in.header.frame_id;
    moveFromROSMsg(req->cloud_in, *temp_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    pass_x.setInputCloud(temp_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(
      static_cast<float>(x_min_),
      static_cast<float>(x_max_));
    pass_x.filter(*temp_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    pass_y.setInputCloud(temp_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(
      static_cast<float>(y_min_),
      static_cast<float>(y_max_));
    pass_y.filter(*temp_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud(temp_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(
      static_cast<float>(z_min_),
      static_cast<float>(z_max_));
    pass_z.filter(*temp_cloud);

    // preseve the TF frame in the output
    toROSMsg(*temp_cloud, res->cloud_out);
    res->cloud_out.header.frame_id = std::move(frame_id);

    return true;
  }

  /**
   * Checks whether the dimensions provided as parameters are valid,
   * If not, then then this message prints an error message and returns
   * false.
   *
   * Note: This function must be called after all the values are initialized.
   *
   * Side-effects: this program uses the state of `this` and prints to std::stderr.
   * @returns
   *  a boolean indicating whether the variables are valid.
   *
   */
  bool check_for_valid_limits() const
  {
    // Checks to make sure all inputs have a positive size
    // NOTE: all comparisons over NaN are implitely false,
    // so this condition catches this case.
    if (
      x_max_ - x_min_ > 0.0 &&
      y_max_ - y_min_ > 0.0 &&
      z_max_ - z_min_ > 0.0)
    {
      return true;
    }

    std::stringstream message;
    message << "Passthrough filter bounds do produce a positive, "
            << "non-zero, non-NaN size:\n"
            << "     X Dimension: " << (x_max_ - y_min_) << "\n"
            << "     Y Dimension: " << (y_max_ - y_min_) << "\n"
            << "     Z Dimension: " << (z_max_ - z_min_) << "\n";

    RCLCPP_FATAL_STREAM(get_logger(), message.str());

    return false;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PassthroughFilterService>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
