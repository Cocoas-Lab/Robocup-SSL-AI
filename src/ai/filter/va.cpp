
#include <cmath>
#include <limits>
#include <chrono>
#include <boost/math/constants/constants.hpp>

#include "ai/model/robot.hpp"
#include "ai/util/math/angle.hpp"
#include "va.hpp"

namespace ai {
namespace filter {

template <>
model::robot va<model::robot>::va::update(const model::robot& _value,
                                          util::TimePointType _time) {
  auto result = _value;

  if (prevTime_ == TimePointType::min()) {
    // 初めてupdateが呼ばれたときは速度加速度を計算しない
    result.vx(0);
    result.vy(0);
    result.omega(0);
    result.ax(0);
    result.ay(0);
  } else {
    namespace bmc = boost::math::double_constants;

    // 前回呼ばれたときからの経過時間
    const auto dt = std::chrono::duration<double>{_time - prevTime_}.count();
    // 非常に短い間隔でupdateが呼び出されたら直前の値を返す
    // (ゼロ除算の原因になるので)
    if (std::abs(dt) < std::numeric_limits<double>::epsilon()) return prevState_;

    // 速度の計算
    result.vx((result.x() - prevState_.x()) / dt);
    result.vy((result.y() - prevState_.y()) / dt);

    // 角速度の計算
    // 境界で大きな値になるのを防ぐために, 偏差がpi以下かそうでないかで処理を変える
    const auto dtheta =
        util::math::wrapTo2pi(result.theta()) - util::math::wrapTo2pi(prevState_.theta());
    if (std::abs(dtheta) < bmc::pi) {
      result.omega(dtheta / dt);
    } else {
      const auto dtheta2 =
          util::math::wrapToPi(result.theta()) - util::math::wrapToPi(prevState_.theta());
      result.omega(dtheta2 / dt);
    }

    // 加速度の計算
    result.ax((result.vx() - prevState_.vx()) / dt);
    result.ay((result.vy() - prevState_.vy()) / dt);
  }

  prevState_ = result;
  prevTime_  = _time;
  return result;
}

} // namespace filter
} // namespace ai
