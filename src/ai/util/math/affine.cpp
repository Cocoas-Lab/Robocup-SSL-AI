#include "angle.hpp"
#include "affine.hpp"
#include "toVector.hpp"

namespace ai {
namespace util {
namespace math {

model::ball transform(const Eigen::Affine3d& _matrix, const model::ball& _ball) {
  // 回転方向は必要ないので
  Eigen::Vector3d p{};
  p.head<2>() = util::math::position(_ball);

  // matrixで座標の変換をする
  const auto r = _matrix * p;

  auto result = _ball;
  result.x(r.x());
  result.y(r.y());
  return result;
}

model::robot transform(const Eigen::Affine3d& _matrix, const model::robot& _robot) {
  // matrixで座標の変換をする
  const auto r = _matrix * util::math::position3d(_robot);

  auto result = _robot;
  result.x(r.x());
  result.y(r.y());
  result.theta(util::math::wrapTo2pi(r.z()));
  return result;
}

} // namespace math
} // namespace util
} // namespace ai
