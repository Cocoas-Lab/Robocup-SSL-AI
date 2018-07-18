#ifndef AI_UTIL_MATH_ANGLE_HPP_
#define AI_UTIL_MATH_ANGLE_HPP_

#include <boost/math/constants/constants.hpp>
#include <cmath>

namespace ai {
namespace util {
namespace math {

template <class T, std::enable_if_t<std::is_floating_point<T>::value, std::nullptr_t> = nullptr>
T wrapTo2pi(T r) {
  using boost::math::constants::two_pi;

  auto wrapped = std::fmod(r, two_pi<T>());

  if (r < 0) {
    wrapped += two_pi<T>();
  }
  return wrapped;
}

/// @brief	-pi<r<=piに正規化
template <class T, std::enable_if_t<std::is_floating_point<T>::value, std::nullptr_t> = nullptr>
T wrapToPi(T r) {
  using boost::math::constants::pi;
  using boost::math::constants::two_pi;

  auto wrapped = std::fmod(r, two_pi<T>());

  if (wrapped > pi<T>()) {
    wrapped -= two_pi<T>();
  } else if (wrapped <= -pi<T>()) {
    wrapped += two_pi<T>();
  }
  return wrapped;
}
} // namespace math
} // namespace util
} // namespace ai
#endif
