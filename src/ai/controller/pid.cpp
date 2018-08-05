#include <boost/math/constants/constants.hpp>
#include <boost/algorithm/clamp.hpp>
#include <cmath>

#include "ai/util/math/angle.hpp"
#include "pid.hpp"

namespace ai {
namespace controller {

// model::command::velocityの四則演算のオーバロード
const velocity operator+(const velocity& _vel1, const velocity& _vel2) {
  return {_vel1.vx + _vel2.vx, _vel1.vy + _vel2.vy, _vel1.omega + _vel2.omega};
}

const velocity operator-(const velocity& _vel1, const velocity& _vel2) {
  return {_vel1.vx - _vel2.vx, _vel1.vy - _vel2.vy, _vel1.omega - _vel2.omega};
}

const velocity operator*(const double& _c, const velocity& _vel) {
  return {_c * _vel.vx, _c * _vel.vy, _c * _vel.omega};
}

const velocity operator*(const velocity& _vel1, const velocity& _vel2) {
  return {_vel1.vx * _vel2.vx, _vel1.vy * _vel2.vy, _vel1.omega * _vel2.omega};
}

const velocity operator/(const velocity& _vel, const double& _c) {
  return {_vel.vx / _c, _vel.vy / _c, _vel.omega / _c};
}

pid::pid(double _cycle) : base(maxVelocity_), cycle_(_cycle) {
  for (int i = 0; i < 2; i++) {
    up_[i] = {0.0, 0.0, 0.0};
    ui_[i] = {0.0, 0.0, 0.0};
    ud_[i] = {0.0, 0.0, 0.0};
    u_[i]  = {0.0, 0.0, 0.0};
    e_[i]  = {0.0, 0.0, 0.0};
  }
}

void pid::velocityLimit(const double _limit) {
  base::velocityLimit(std::min(_limit, maxVelocity_));
}

velocity pid::update(const model::robot& _robot, const position& _setpoint) {
  robot_ = _robot;
  // 位置偏差
  position ep;
  ep.x     = _setpoint.x - robot_.x();
  ep.y     = _setpoint.y - robot_.y();
  ep.theta = util::math::wrapToPi(_setpoint.theta - robot_.theta());

  double speed    = std::hypot(ep.x, ep.y);
  double direction = util::math::wrapToPi(std::atan2(ep.y, ep.x) - robot_.theta());

	// 計算用にゲイン再計算
	auto kp = model::command::velocity{kp_[0], kp_[0], kp_[1]};
	auto ki = model::command::velocity{ki_[0], ki_[0], ki_[1]};
	auto kd = model::command::velocity{kd_[0], kd_[0], kd_[1]};

  e_[0].vx         = speed * std::cos(direction);
  e_[0].vy         = speed * std::sin(direction);
  e_[0].omega      = ep.theta;

  up_[0] = kp * e_[0];
  ui_[0] = cycle_ * ki * (e_[0] + e_[1]) / 2 + ui_[1];
  ud_[0] = 2 * kd * (e_[0] - e_[1]) / cycle_ - ud_[1];
  u_[0]  = up_[0] + ui_[0] + ud_[0];

  // 入力制限計算
  limitation();

  return u_[0];
}

velocity pid::update(const model::robot& _robot, const velocity& _setpoint) {
  robot_           = _robot;
  double direction = std::atan2(_setpoint.vy, _setpoint.vx);
  double speed     = std::hypot(_setpoint.vx, _setpoint.vy);
  velocity vel;
  vel.vx    = speed * std::cos(direction - robot_.theta());
  vel.vy    = speed * std::sin(direction - robot_.theta());
  vel.omega = _setpoint.omega;

  // 現在偏差
  e_[0].vx    = vel.vx - robot_.vx();
  e_[0].vy    = vel.vy - robot_.vy();
  e_[0].omega = vel.omega - robot_.omega();

  // 計算用にゲイン再計算
  velocity kp = model::command::velocity{kp_[0], kp_[0], kp_[1]};
  velocity ki = model::command::velocity{ki_[0], ki_[0], ki_[1]};
  velocity kd = model::command::velocity{kd_[0], kd_[0], kd_[1]};

  // 双一次変換
  // s=(2/T)*(Z-1)/(Z+1)としてPIDcontrollerを離散化
  // C=Kp+Ki/s+Kds
  up_[0] = kp * e_[0];
  ui_[0] = cycle_ * ki * (e_[0] + e_[1]) / 2 + ui_[1];
  ud_[0] = 2 * kd * (e_[0] - e_[1]) / cycle_ - ud_[1];
  u_[0]  = u_[1] + cycle_ * (up_[0] + ui_[0] + ud_[0]);

  // 入力制限計算
  limitation();

  return u_[0];
}

void pid::limitation() {
  // 加速度，速度制限
  using boost::algorithm::clamp;
  double uAngle     = std::atan2(u_[0].vy, u_[0].vx); // 今回指令速度の方向
  double uSpeed     = std::hypot(u_[0].vx, u_[0].vy); // 今回指令速度の大きさ
  double robotSpeed = std::hypot(robot_.vx(), robot_.vy());
  double deltaSpeed = uSpeed - robotSpeed; // 速さ偏差(今回指令とロボット速さの差)
  // 速度に応じて加速度を変化(初動でのスリップ防止)
  // 制限加速度計算
  double optimizedAccel =
      robotSpeed * (maxAcceleration_ - minAcceleration_) / reachSpeed_ + minAcceleration_;
  optimizedAccel = clamp(optimizedAccel, minAcceleration_, maxAcceleration_);
  // 加速度制限
  if (deltaSpeed / cycle_ > optimizedAccel &&
      std::abs(uSpeed) > std::abs(robotSpeed)) { // +制限加速度超過
    uSpeed = robotSpeed + (optimizedAccel * cycle_);
  } else if (deltaSpeed / cycle_ < -optimizedAccel &&
             std::abs(uSpeed) > std::abs(robotSpeed)) { // -制限加速度超過
    uSpeed = robotSpeed - (optimizedAccel * cycle_);
  }
  // 速度制限
  uSpeed = clamp(uSpeed, 0.0, velocityLimit_);

  u_[0].vx = uSpeed * std::cos(uAngle); // 成分速度再計算
  u_[0].vy = uSpeed * std::sin(uAngle);

  // 値の更新
  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  u_[1]  = u_[0];
  e_[1]  = e_[0];
}

} // namespace controller
} // namespace ai
