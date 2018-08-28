#include <cmath>
#include "predictor.hpp"

namespace ai {
namespace controller {
namespace decision {

predictor::predictor(const double _cycle, const double _zeta, const double _omega)
    : cycle_(_cycle), zeta_(_zeta), omega_(_omega) {
  for (int i = 0; i < 7; i++) {
    u_[i] = Eigen::Vector3d::Zero();
  }
}

Eigen::Matrix3d predictor::interpolate(const model::robot& _robot, const Eigen::Vector3d& _u) {
  u_[0] = _u; // 受け取った制御入力を最新入力として

  // 受け取ったロボットの状態(無駄時間分の遅れ含む)
  // 計算しやすいように,ベクトルに変換
  Eigen::Vector3d prePosition(_robot.x(), _robot.y(), _robot.theta());
  Eigen::Vector3d preVelocity(_robot.vx(), _robot.vy(), _robot.omega());
  Eigen::Vector3d preAcceleration(_robot.ax(), _robot.ay(), 0);

  auto nowPosition     = prePosition;
  auto nowVelocity     = preVelocity;
  auto nowAcceleration = preAcceleration;

  // 1ループ毎に当時の制御入力によって1フレーム分の補間をする
  for (int i = 6; i >= 0; i--) {
    // p=p+v*dt
    nowPosition = nowPosition + cycle_ * preVelocity;
    // v=v+a*dt
    nowVelocity = nowVelocity + cycle_ * preAcceleration;
    // a=a+omega^2*v*dt-2*zeta*omega*a+omega^2*u
    nowAcceleration = nowAcceleration + cycle_ * (-std::pow(omega_, 2) * preVelocity -
                                                  2 * zeta_ * omega_ * preAcceleration +
                                                  std::pow(omega_, 2) * _u);

    // 更新
    if (i != 0) {
      u_[i] = u_[i - 1];
    }
    prePosition     = nowPosition;
    preVelocity     = nowVelocity;
    preAcceleration = nowAcceleration;
  }

  // 返すために変換
  Eigen::Matrix3d nowState;
  nowState.col(0) = nowPosition;
  nowState.col(1) = nowVelocity;
  nowState.col(2) = nowAcceleration;

  return nowState;
}

} // namespace decision
} // namespace controller
} // namespace ai
