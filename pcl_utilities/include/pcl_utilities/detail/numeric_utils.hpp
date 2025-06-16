#ifndef ROBOT_COMMON_3D_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_DETAIL_NUMERIC_UTILS_H_
#define ROBOT_COMMON_3D_PCL_UTILITIES_INCLUDE_PCL_UTILITIES_DETAIL_NUMERIC_UTILS_H_

#include <string>  // std::to_string
#include <limits>  //std::numeric_limits
#include <stdexcept>  // std::overflow_error, std::underflow_error
#include <type_traits>  // std::is_same_v, std::common_type_t, std::make_unsigned
#include <utility>  // std::pair

namespace pcl_utilities::detail
{

/**
 * @brief Let T and U be numeric types. Returns the intersection of ranges of T
 * and U as T.
 * @tparam T a number type with std::numeric_limits specialization
 * @tparam U the number type of the resulting ranges
 * @return A pair containing the min and max possible ranges of the numeric
 *   type T on the numeric type U.
 */
template<typename T, typename U>
constexpr std::pair<T, T> map_numeric_range()
{
  using T_limits = std::numeric_limits<T>;
  using U_limits = std::numeric_limits<U>;

  // if both have the same sign, (+) (+) or (-) (-)
  if constexpr (
    T_limits::is_signed == U_limits::is_signed ||
    !T_limits::is_integer || !U_limits::is_integer ||
    std::is_same_v<T, bool>|| std::is_same_v<U, bool>)
  {
    using Num = std::common_type_t<T, U>;
    bool t_min_gt = static_cast<Num>(T_limits::lowest()) >= static_cast<Num>(U_limits::lowest());
    bool t_max_lt = static_cast<Num>(T_limits::max()) <= static_cast<Num>(U_limits::max());

    return std::pair{
      t_min_gt ? T_limits::lowest() : static_cast<T>(U_limits::lowest()),
      t_max_lt ? T_limits::max() : static_cast<T>(U_limits::max())};
  } else {  // if differing signs: (+) (-), (-) (+), make type signed
    using Num = std::common_type_t<std::make_unsigned_t<T>, std::make_unsigned_t<U>>;
    auto t_max_lt = static_cast<Num>(T_limits::max()) <= static_cast<Num>(U_limits::max());

    return std::pair{T{}, t_max_lt ? T_limits::max() : static_cast<T>(U_limits::max())};
  }
}

/**
 * @brief If both T, U are numeric and number can be safely converted to type T
 *   without overflow, return T. If the conversion results in overflow, throw.
 *   If the types are non-numeric, use a regular static cast
 * type, throws otherwise. Regular static cast if a type is non-numeric
 * @tparam T the type of resulting cast
 * @tparam U the source type of the input number
 * @param number the source number type of T
 * @return the resulting type of type U
 * @throw std::underflow if the number is below the minimum range of type T
 * @throw std::overflow if the number is above the maximum value of type T
 */
template<typename T, typename U>
T narrowing_cast(U number)
{
  if constexpr (!std::is_same_v<T, U>|| !std::is_arithmetic_v<T>|| !std::is_arithmetic_v<U>) {
    static constexpr auto min_max = map_numeric_range<U, T>();
    if (number > min_max.second) {
      throw std::overflow_error(
        "Unsafe numeric conversion will result in overflow: " +
        std::to_string(number) + " is above " + std::to_string(min_max.first));
    } else if (number < min_max.first) {
      throw std::underflow_error(
        "Unsafe numeric conversion will result in underflow: " +
        std::to_string(number) + " is below " + std::to_string(min_max.second));
    }
  }

  return static_cast<T>(number);
}

}  // namespace pcl_utilities::detail
#endif
