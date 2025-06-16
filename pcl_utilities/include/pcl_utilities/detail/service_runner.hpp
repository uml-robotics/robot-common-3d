#ifndef ROBOT_COMMON_3D_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_DETAIL_SERVICE_RUNNER_H_
#define ROBOT_COMMON_3D_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_DETAIL_SERVICE_RUNNER_H_

#include "rmw/qos_profiles.h"  // rmw_qos_profile_services_default

#include <any> // std::any
#include <functional>  // std::function
#include <memory>  // std::make_unique, std::unique_ptr
#include <type_traits>                                   // std::decay_t<T>
#include <utility>  // std::any, std::forward<T>, std::move<T>, std::tuple<Ts...>, std::apply<T, Tuple...>
#include <vector>  // std::vector<T, A>

#include "rclcpp/executors.hpp"  // rclcpp::spin
#include "rclcpp/executors/multi_threaded_executor.hpp"  // rclcpp::Executors::MultiThreadedExecutor
#include "rclcpp/node.hpp" // rclcpp::Node
#include "rclcpp/service.hpp" // rclcpp::Service
#include "rclcpp/callback_group.hpp"

#include "pcl_utilities/detail/numeric_utils.hpp"  // narrowing_cast<T, U>
#include "pcl_utilities/detail/param.hpp"  // detail::GenericParam, Param<T>

namespace pcl_utilities::detail
{

/**
 * @brief Generic wraper around a service and node.
 * @details This class represents a generic service that is meant to run all
 * pcl utility nodes, this script is designed to contain all needed features.
 *
 * @note This is not thread safe when if modiying the service concurrently.
 *   As long as you do not modify the service runner while spinning, this
 *   is guarenteed to be safe.
 **/
template<typename SrvT>
class ServiceRunner
{
  using Request = typename SrvT::Request;
  using Response = typename SrvT::Response;
  using ParamVector = std::vector<std::unique_ptr<GenericParam>>;
  using CallbackFunc = std::function<void (const Request &, Response &)>;
  using ParamFunc = std::function<ParamVector(Request &)>;

  Request dummy_msg_{};
  ParamVector params_{};
  ParamFunc param_func_{};
  CallbackFunc callback_{};

  rclcpp::Node::SharedPtr node_{};
  typename rclcpp::Service<SrvT>::SharedPtr service_{};
  rclcpp::CallbackGroup::SharedPtr callback_group_{};

  void service_callback(typename Request::SharedPtr request, typename Response::SharedPtr response)
  {
    // Message may be loaned, therefore, save and restore old
    // values onto the class.

    std::vector<std::any> old_values;
    ParamVector params;

    old_values.reserve(params_.size());
    if (param_func_ && node_->get_parameter("use_params").get_value<bool>()) {
      params = param_func_(*request);
    }

    for (auto && param : params) {
      old_values.push_back(param->move_into_any());
      param->from_parameter_value(node_->get_parameter(param->get_name()).get_parameter_value());
    }

    // Display error if callback cannot be called
    try {
      callback_(*request, *response);
    } catch (std::exception & error) {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(), "Failed to execute the service call \""
          << service_->get_service_name() << "\" with error \"" << error.what()
          << "\"\n");
    }

    // Restore old values into request
    for (size_t i = 0; i < params.size(); ++i) {
      params[i]->from_any(std::move(old_values[i]));
    }
  }

public:
  ServiceRunner(rclcpp::Node::SharedPtr node)
  : node_{node}
  {
    node_->declare_parameter("use_params", true);
  }

  /**
   * @brief Constructor that fowards all arguments to construct a node
  */
  template<typename ... Args>
  ServiceRunner(Args &&... args)
  : ServiceRunner(rclcpp::Node::make_shared(std::forward<Args>(args)...))
  {
  }

  rclcpp::Node::SharedPtr get_node() {return node_;}

  /** @brief Run node and all services in single threaded manner
   **/
  void spin_one_thread() {rclcpp::spin(node_);}

  /** @brief Run node and all services in multi threaded manner
   **/
  void spin_multi_thread()
  {
    rclcpp::executors::MultiThreadedExecutor executor{};
    executor.add_node(node_);
    executor.spin();
  }

  /**
   * @brief Creates a service with the request msg feild updated with parameter
   *values
   * @param name The name of the service to expose parameter as
   * @param func The callback function that will be provided to the serviced
   **/
  void define_service(const std::string & name, CallbackFunc func)
  {
    callback_ = func;

    auto callback = std::bind(
      &ServiceRunner<SrvT>::service_callback, this, std::placeholders::_1, std::placeholders::_2);

    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_ = node_->create_service<SrvT>(
      name, std::move(callback), rmw_qos_profile_services_default, callback_group_);
  }

  /**
   * @brief Extracts parameter definitions from a request to be exposed
   * @param typed_params_func function that returns the parameters
   * @tparam Func (Type: `auto (Request&) -> std::tuple<Param<StaticType>...>
   *   A function that returns a std::tuple of Param's, these params will
   * automatically store the value of the ros2 parameter into a request msg,
   * then restore the old field value after returning.
   *
   * @note The pointer in `Param` must be valid for the entire duration
   *  of the provided request.
   */
  template<typename Func>
  void expose_request_parameters(Func && typed_params_func)
  {
    /*
      Perform type erasure on the typed result of typed_param_func
      Steps:
        1. Visit each static Param of the lambda with `std::apply` and pack
      expansion
        2. Make it a unique_ptr such that it can safely cast to the
           type erased GenericParam without slicing.
        3. Append it to the vector
    */
    auto param_func = [&typed_params_func](typename SrvT::Request & request) {
        ParamVector params;
        std::apply(
          [&params](auto... typed_params) {
            (params.push_back(std::move(typed_params).erase_type()), ...);
          },
          typed_params_func(request));

        return params;
      };

    // remove any old parameters
    for (auto && param : params_) {
      node_->undeclare_parameter(param->get_name());
    }

    params_ = param_func(dummy_msg_);     // Defer storing to std::function to allow inlining
    param_func_ = std::move(param_func);  // store the typed erased function

    for (auto && param : params_) {
      node_->declare_parameter(
        param->get_name(), param->to_parameter_value(), param->get_constraint());
    }
  }
};

}  // namespace pcl_utilities::detail

namespace pcl_utilities
{
using pcl_utilities::detail::ServiceRunner;
}  // namespace pcl_utilities

#endif
