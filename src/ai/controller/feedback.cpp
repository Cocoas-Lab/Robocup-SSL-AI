#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "ai/util/math/angle.hpp"
#include "feedback.hpp"

namespace ai {
namespace controller {

using boost::math::constants::half_pi;
using boost::math::constants::pi;

const double feedback::k_     = 49.17;
const double feedback::zeta_  = 1.0;
const double feedback::omega_ = 49.17;
const double feedback::vMax_  = 5000.0;

feedback::feedback(double _cycle, const model::world& _world)
    : base(vMax_),
      cycle_(_cycle),
      world_(_world),
      generator_(cycle_),
      predictor_(cycle_, zeta_, omega_) {
  // 状態フィードバックゲイン
  // (s+k)^2=s^2+2ks+k^2=0
  // |sI-A|=s^2+(2ζω-k2ω^2)s+ω^2-k1ω^2
  // 2k=2ζω-k2ω^2,k^2=ω^2-k1ω^2
  double k1 = 1.0 - std::pow(k_, 2) / std::pow(omega_, 2);
  double k2 = 2.0 * (zeta_ * omega_ - k_) / std::pow(omega_, 2);
  kp_       = {k1, k1, 0.0};
  ki_       = {0.0, 0.0, 0.0};
  kd_       = {k2, k2, 0.0};
  for (int i = 0; i < 2; i++) {
    up_[i] = Eigen::Vector3d::Zero();
    ui_[i] = Eigen::Vector3d::Zero();
    ud_[i] = Eigen::Vector3d::Zero();
    u_[i]  = Eigen::Vector3d::Zero();
    e_[i]  = Eigen::Vector3d::Zero();
  }
}

void feedback::velocityLimit(const double _limit) {
  base::velocityLimit(std::min(_limit, vMax_));
}

velocity feedback::update(const model::robot& _robot, const position& _setpoint) {
  calcRegulator(_robot);
  Eigen::Vector3d set    = {_setpoint.x, _setpoint.y, _setpoint.theta};
  Eigen::Vector3d eP     = set - estimatedRobot_.col(0);
  eP.z()                 = util::math::wrapToPi(eP.z());
  Eigen::Vector3d deltaP = convert(eP, estimatedRobot_(2, 0));
  double targetAngle     = std::atan2(deltaP.y(), deltaP.x());

  Eigen::Vector3d target;
  double targetSpeed = std::hypot(deltaP.x(), deltaP.y());
  auto tmp           = generator_.generate(position{deltaP.x(), deltaP.y(), 0}, stable_);
  target.x()         = tmp.vx;
  target.y()         = tmp.vy;
  target.z()         = 2.0 * util::math::wrapToPi(deltaP.z());

  calcOutput(target, targetAngle);

  return velocity{u_[0].x(), u_[0].y(), u_[0].z()};
}

velocity feedback::update(const model::robot& _robot, const velocity& _setpoint) {
  calcRegulator(_robot);

  Eigen::Vector3d set    = {_setpoint.vx, _setpoint.vy, _setpoint.omega};
  Eigen::Vector3d target = convert(set, estimatedRobot_(2, 0));
  double targetAngle     = std::atan2(target.y(), target.x());
  auto tmp               = generator_.generate(velocity{target.x(), target.y(), 0}, stable_);

  target.x() = tmp.vx;
  target.y() = tmp.vy;
  target.z() = set.z();

  calcOutput(target, targetAngle);

  return velocity{u_[0].x(), u_[0].y(), u_[0].z()};
}

void feedback::calcRegulator(const model::robot& _robot) {
  // 前回制御入力をフィールド基準に座標変換
  double uDirection    = std::atan2(u_[1].y(), u_[1].x());
  Eigen::Vector3d preU = Eigen::AngleAxisd(uDirection, Eigen::Vector3d::UnitZ()) * u_[1];

  // smith_predictorでvisionの遅れ時間の補間
  estimatedRobot_ = predictor_.interpolate(_robot, preU);

  // ロボット速度を座標変換
  e_[0] = convert(estimatedRobot_.col(1), estimatedRobot_(2, 0));

  // 双一次変換
  // s=(2/T)*(Z-1)/(Z+1)としてPIDcontrollerを離散化
  // C=Kp+Ki/s+Kds
  up_[0] = kp_.array() * e_[0].array();
  ui_[0] = cycle_ * ki_.array() * (e_[0].array() + e_[1].array()) / 2.0 + ui_[1].array();
  ud_[0] = 2.0 * kd_.array() * (e_[0].array() - e_[1].array() / cycle_ - ud_[1].array());

  u_[0] = up_[0] + ui_[0] + ud_[0];
}

Eigen::Vector3d feedback::convert(const Eigen::Vector3d& _raw, const double _robotTheta) {
  Eigen::Vector3d target = Eigen::AngleAxisd(-_robotTheta, Eigen::Vector3d::UnitZ()) * _raw;
  return target;
}
// 出力計算及び後処理
void feedback::calcOutput(Eigen::Vector3d _target, double _targetAngle) {
  // 速度が大きいときに角速度が大きくなりすぎないように
  double omegaLimit = pi<double>() * std::exp(-std::hypot(_target.x(), _target.y()) / 2000.0);
  _target.z()       = std::clamp(_target.z(), -omegaLimit, omegaLimit);

  // スピンしてる(角速度が大きすぎる)ときは速度落とす
  if (estimatedRobot_(2, 1) > 10) {
    _target = Eigen::Vector3d::Zero();
  }

  // ロボット入力計算
  u_[0] = u_[0] + (std::pow(k_, 2) / std::pow(omega_, 2)) * _target;
  // nanが入ったら前回入力を今回値とする
  if (std::isnan(u_[0].x()) || std::isnan(u_[0].y()) || std::isnan(u_[0].z())) {
    u_[0] = u_[1];
  }

  double vxMax         = velocityLimit_;
  double vxMin         = velocityLimit_;
  double vyMax         = velocityLimit_;
  double vyMin         = velocityLimit_;
  double marginOutside = 500.0;
  double marginInside  = 1500.0;
  double width         = marginOutside + marginInside;
  bool flag            = false;
  // フィールドに対して外に出そうなやつは速度制限を強める
  if (estimatedRobot_(0, 0) > world_.field().xMax() - marginInside) {
    vxMax *= (world_.field().xMax() + marginOutside - estimatedRobot_(0, 0)) / width;
    flag  = true;
    vxMax = std::clamp(vxMax, 0.0, vxMax);
  }
  if (estimatedRobot_(0, 0) < world_.field().xMin() + marginInside) {
    vxMin *= (world_.field().xMin() - marginOutside - estimatedRobot_(0, 0)) / width;
    flag  = true;
    vxMin = std::clamp(vxMin, vxMin, 0.0);
  }
  if (estimatedRobot_(1, 0) > world_.field().yMax() - marginInside) {
    vyMax *= (world_.field().yMax() + marginOutside - estimatedRobot_(1, 0)) / width;
    flag  = true;
    vyMax = std::clamp(vyMax, 0.0, vyMax);
  }
  if (estimatedRobot_(1, 0) < world_.field().yMin() + marginInside) {
    vyMin *= (world_.field().yMin() - marginOutside - estimatedRobot_(1, 0)) / width;
    flag  = true;
    vyMin = std::clamp(vyMin, vyMin, 0.0);
    if (vyMin > 0) {
      vyMin = 0.0;
    }
  }
  if (flag && std::hypot(_target.x(), _target.y()) > 2000.0) {
    _target.z() = 0.0;
  }
  double ratio = 1.0;
  if (std::abs(estimatedRobot_(2, 1)) > pi<double>()) {
    //  ratio = pi<double>() / std::abs(estimated_robot_(2, 1));
  }
  double posiX = ratio;
  double negaX = ratio;
  double posiY = ratio;
  double negaY = ratio;
  // 各方向の速度制限線が作る四角形がロボットの速度制限
  // 2直線の交点を求めることでロボットのxyに対しての速度制限計算
  /*if (estimated_robot_(2, 0) < -half_pi<double>()) {
    posi_x *= find_cross_point(vx_min, vy_max, estimated_robot_(2, 0) + half_pi<double>());
    posi_y *= find_cross_point(vx_min, vy_min, estimated_robot_(2, 0));
    nega_x *= find_cross_point(vx_max, vy_min, estimated_robot_(2, 0) + half_pi<double>());
    nega_y *= find_cross_point(vx_max, vy_max, estimated_robot_(2, 0));
  } else if (estimated_robot_(2, 0) < 0) {
    posi_x *= find_cross_point(vx_min, vy_min, estimated_robot_(2, 0) - half_pi<double>());
    posi_y *= find_cross_point(vx_max, vy_min, estimated_robot_(2, 0));
    nega_x *= find_cross_point(vx_max, vy_max, estimated_robot_(2, 0) + half_pi<double>());
    nega_y *= find_cross_point(vx_min, vy_max, estimated_robot_(2, 0));
  } else if (estimated_robot_(2, 0) < half_pi<double>()) {
    posi_x *= find_cross_point(vx_max, vy_min, estimated_robot_(2, 0) - half_pi<double>());
    posi_y *= find_cross_point(vx_max, vy_max, estimated_robot_(2, 0));
    nega_x *= find_cross_point(vx_min, vy_max, estimated_robot_(2, 0) + half_pi<double>());
    nega_y *= find_cross_point(vx_min, vy_min, estimated_robot_(2, 0));
  } else {
    posi_x *= find_cross_point(vx_max, vy_max, estimated_robot_(2, 0) - half_pi<double>());
    posi_y *= find_cross_point(vx_min, vy_max, estimated_robot_(2, 0));
    nega_x *= find_cross_point(vx_min, vy_min, estimated_robot_(2, 0) - half_pi<double>());
    nega_y *= find_cross_point(vx_max, vy_min, estimated_robot_(2, 0));
  }*/
  // 速度制限
  u_[0].x() = std::clamp(u_[0].x(), -velocityLimit_, velocityLimit_);
  u_[0].y() = std::clamp(u_[0].y(), -velocityLimit_, velocityLimit_);

  if (std::abs(estimatedRobot_(0, 0)) > world_.field().xMax() + marginOutside ||
      std::abs(estimatedRobot_(1, 0)) > world_.field().yMax() + marginOutside) {
    double toCenterAngle = std::atan2(-estimatedRobot_(1, 0), -estimatedRobot_(0, 0));
    if (std::abs(util::math::wrapToPi(toCenterAngle - (_targetAngle + estimatedRobot_(2, 0)))) >
        half_pi<double>()) {
      //    	u_[0].x() = 0.0;
      //  	u_[0].y() = 0.0;
    }
  }
  // 最終的に速度ベクトルが目標通りになるように
  double speed = std::hypot(u_[0].x(), u_[0].y());
  u_[0].x()    = speed * std::cos(_targetAngle);
  u_[0].y()    = speed * std::sin(_targetAngle);

  // 値の更新
  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  u_[1]  = u_[0];
  e_[1]  = e_[0];
}

double feedback::findCrossPoint(const double _x1, const double _y2, const double _angle) {
  // 2点から直線の式を求め,交点を求める
  // (y_1,x_2はゼロ)
  double x;
  double y;
  if (std::abs(_angle) == half_pi<double>()) {
    return _y2;
  }
  if (_x1 == 0.0 || _y2 == 0.0) {
    return 0.0;
  }
  x = _x1 * _y2 / (_x1 * std::tan(_angle) + _y2);
  y = -x * _y2 / _x1 + _y2;

  // 原点から交点までの距離を返す
  return std::hypot(x, y);
}

} // namespace controller
} // namespace ai
