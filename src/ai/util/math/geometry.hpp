#ifndef AI_UTIL_MATH_GEOMETRY_HPP_
#define AI_UTIL_MATH_GEOMETRY_HPP_

#include <boost/math/constants/constants.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include "ai/util/math.hpp"

namespace ai {
namespace util {
namespace math {
/// @brief     ある直線の始点と終点が与えられたとき,終点から左右に任意の長さ分ずらした2点を返す
/// @param apex ある直線についての始点
/// @param middle_base ある直線についての終点
/// @param shift 終点からずらしたい長さ
/// @return
/// std::tuple<Eigen::Matrix<T,2,1>,Eigen::Matrix<T,2,1>>{終点から右にずらした点,終点から左にずらした点}
template <class T, std::enable_if_t<std::is_floating_point<T>::value, std::nullptr_t> = nullptr>
std::tuple<Eigen::Matrix<T, 2, 1>, Eigen::Matrix<T, 2, 1>> calcIsoscelesVertexes(
    const Eigen::Matrix<T, 2, 1>& _apex, const Eigen::Matrix<T, 2, 1>& _middleBase, T _shift) {
  const Eigen::Matrix<T, 2, 1> move{_apex};

  //計算の為に中心にずらした場合の座標
  Eigen::Matrix<T, 2, 1> afterApex{_apex - move};

  Eigen::Matrix<T, 2, 1> afterMiddleBase{_middleBase - move};

  // x軸から角度
  const auto alpha = util::wrapTo2pi(
      std::atan2(afterMiddleBase.y() - afterApex.y(), afterMiddleBase.x() - afterApex.x()));
  //回転行列
  const Eigen::Rotation2D<T> rotate(alpha);

  after_middle_base.x() = (afterMiddleBase - afterApex).norm();
  after_middle_base.y() = 0.0;

  //移動した先での仮の座標
  const Eigen::Matrix<T, 2, 1> tmp1(afterMiddleBase.x(), _shift);
  const Eigen::Matrix<T, 2, 1> tmp2(tmp1.x(), tmp1.y() * (-1));

  //回転した後の正しい座標
  return std::make_tuple((rotate * tmp1) + move, (rotate * tmp2) + move);
}
} // namespace math
} // namespace util
} // namespace ai
#endif
