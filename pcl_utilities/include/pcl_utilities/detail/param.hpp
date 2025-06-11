#ifndef ROBOT_COMMON_3D_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_DETAIL_PARAM_H_
#define ROBOT_COMMON_3D_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_DETAIL_PARAM_H_

#include <any>  // std::any
#include <memory>  // std::make_shared<T>

#include <string>  // std::string
#include <type_traits>  // std::conditional_t<T>, std::is_floating_point_v<T>, std::is_integral_v<T>, std::is_same_v<T, U>,
#include <tuple>  // std::tie<Args...>
#include <utility>  // std::move<T>, std::forward<T>
#include <vector>  // std::vector<T>

#include <rclcpp/parameter_value.hpp> // rclcpp::ParameterValue

#include <rcl_interfaces/msg/floating_point_range.hpp>  // rcl_interfaces::msg::FloatingPointRange
#include <rcl_interfaces/msg/integer_range.hpp>  // rcl_interfaces::msg::IntegerRange
#include <rcl_interfaces/msg/parameter_descriptor.hpp>  // ParameterDescriptor

#include "pcl_utilities/detail/numeric_utils.hpp"  // map_numeric_range<T, U>, narrowing_cast<T, U>

namespace pcl_utilities::detail
{
/**
 * @brief INTERNAL USE ONLY. Type erased ROS2 parameter reference, designed to handle all
 *   parameter types (and arithmetic types)
 */
class GenericParam
{
protected:
  GenericParam() = default;
  GenericParam(const GenericParam &) = default;
  GenericParam(GenericParam &&) = default;
  GenericParam & operator=(const GenericParam &) = default;
  GenericParam & operator=(GenericParam &&) = default;

public:
  virtual void from_parameter_value(const rclcpp::ParameterValue & parameter) = 0;
  virtual rclcpp::ParameterValue to_parameter_value() = 0;
  virtual const rcl_interfaces::msg::ParameterDescriptor & get_constraint() const = 0;
  virtual const std::string & get_name() const = 0;
  virtual std::any move_into_any() = 0;
  virtual void from_any(std::any &&) = 0;

  virtual ~GenericParam() {}
};

/**
 * @brief Associates a ROS2 Parameter with a service request field.
 *
 * @tparam T type of the request field, cannot be `std::vector<int>`
 *   or `std::vector<float>`
 * @note The lifetime field must live at least as long as this class.
 *   As long as fields are only provided from the given message, this is
 *   guarenteed to be true
 **/
template<typename T>
class Param : public detail::GenericParam
{
  static_assert(!std::is_same_v<T, std::vector<int>>, "T cannot be std::vector<int>");
  static_assert(!std::is_same_v<T, std::vector<float>>, "T cannot be std::vector<float>");

  using ParamT = std::conditional_t<
    std::is_same_v<T, bool>, bool,
    std::conditional_t<
      std::is_integral_v<T>, int64_t, std::conditional_t<std::is_floating_point_v<T>, double, T>>>;

  std::string name_;
  T & value_;
  rcl_interfaces::msg::ParameterDescriptor descriptor_;

public:
  /**
   * @brief If T is numeric, provides a parameter constraint such that
        any set parameter value is guarenteedsafe convertible to T
   * @return Returns a Parameter Descriptor
   **/
  static rcl_interfaces::msg::ParameterDescriptor get_default_constraint()
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    // descriptor.name/type is ignored
    // handle all arithmetic types, constrain types sufficently such
    // that no invalid value may be provided
    if constexpr (std::is_same_v<ParamT, int64_t>) {
      rcl_interfaces::msg::IntegerRange range;
      std::tie(range.from_value, range.to_value) = detail::map_numeric_range<T, int64_t>();
      descriptor.integer_range.push_back(std::move(range));
    } else if constexpr (std::is_same_v<ParamT, double>) {
      rcl_interfaces::msg::FloatingPointRange range;
      std::tie(range.from_value, range.to_value) = detail::map_numeric_range<T, double>();
      descriptor.floating_point_range.push_back(std::move(range));
    }

    return descriptor;
  }

  /**
   * @note The Type T (CTAD) is the type of the request field, cannot be
   *   `std::vector<int>` or `std::vector<float>`
   * @param name The name of the  service
   * @param[in,out] value_ptr A pointer to the field that will be provide
   *  a default value when declaring parameters and be stored into when reading
   * @param descriptor Defaults to a descriptor that safely bounds integer ranges.
   *  You may overwrite this, but you are responsible for ensuring type safety.
   *  See `get_default_contraint` for more information.
  */
  Param(
    const std::string & name, T * value_ptr,
    rcl_interfaces::msg::ParameterDescriptor descriptor = get_default_constraint())
  : name_{name}, value_{*value_ptr}, descriptor_{descriptor} {}

  void from_parameter_value(const rclcpp::ParameterValue & parameter) override
  {
    value_ = detail::narrowing_cast<T>(parameter.get<ParamT>());
  }

  rclcpp::ParameterValue to_parameter_value() final
  {
    return rclcpp::ParameterValue{detail::narrowing_cast<ParamT>(value_)};
  }

  const rcl_interfaces::msg::ParameterDescriptor & get_constraint() const final
  {
    return descriptor_;
  }

  const std::string & get_name() const final {return name_;}

  std::any move_into_any() final {return std::make_any<T>(std::move(value_));}

  void from_any(std::any && value) final {value_ = std::any_cast<T>(std::move(value));}

  std::unique_ptr<GenericParam> erase_type()
  {
    return std::make_unique<Param<T>>(*this);
  }
};

template<typename T>
// Explicit CTAD Guide
Param(T)->Param<T>;

}  // namespace pcl_utilities::detail

namespace pcl_utilities
{
using detail::Param;
}  // namespace pcl_utilities

#endif
