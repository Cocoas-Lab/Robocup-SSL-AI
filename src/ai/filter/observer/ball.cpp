#include "ball.hpp"
#include <cmath>
#include <limits>
#include <iostream>

namespace ai {
namespace filter {
namespace observer {
constexpr double ball::fricCoef_;
constexpr double ball::ballWeight_;
constexpr double ball::ballRadius_;
constexpr double ball::airViscosity_;
constexpr double ball::airRegistance_;
constexpr double ball::friction_;
constexpr double ball::quantLimitX_;
constexpr double ball::quantLimitY_;
constexpr double ball::lambdaObserver_;

ball::ball(const model::ball& _ball, util::TimePointType _time)
    : ball_(_ball), prevTime_(_time) {
  xHat_[0] << std::floor(_ball.x() / quantLimitX_) * quantLimitX_, 0;
  xHat_[1] << std::floor(_ball.y() / quantLimitY_) * quantLimitY_, 0;
  ball_.vx(0);
  ball_.vy(0);
}

model::ball ball::update(const model::ball& _ball, util::TimePointType _time) {
  Eigen::Matrix<double, 2, 2> A;
  static const Eigen::Matrix<double, 1, 2> C(1, 0);
  Eigen::Matrix<double, 2, 1> h;

  // 前回呼び出しからの経過時刻[s]
  auto passedTime = std::chrono::duration<double>(_time - prevTime_).count();
  prevTime_       = _time;

  auto h1 = [](double _fric) { return -2 * lambdaObserver_ - (_fric + airRegistance_); };

  auto h2 = [h1](double _fric) {
    return std::pow(lambdaObserver_, 2) - h1(_fric) * (_fric + airRegistance_);
  };

  auto theta = std::atan2(ball_.vx(), ball_.vy());

  // x軸方向について状態推定
  auto cosTheta = std::cos(theta);

  // μmg < μvの場合、摩擦力fは飽和してμmgに制限される
  //  -> オブザーバゲイン(正確には摩擦係数)を速度に応じて調整することで対応
  //     μmg < μv ... (μmg / v)を新たにμ'とすればμ'vはμmgに制限される
  if (std::abs(friction_ * cosTheta) < std::abs(fricCoef_ * ball_.vx())) {
    // 摩擦が飽和している領域。速度に応じて摩擦係数を調整することで対処
    auto fricCoefTuned = std::abs(friction_ * cosTheta / ball_.vx());

    A << 0, 1, 0, -(fricCoefTuned + airRegistance_);
    h << h1(fricCoefTuned), h2(fricCoefTuned);
  } else {
    // 摩擦が線形の領域。
    A << 0, 1, 0, -(fricCoef_ + airRegistance_);
    h << h1(fricCoef_), h2(fricCoef_);
  }

  // ロストを検出する機構がないので、座標が最大値かどうかで検出を行う
  auto ball = _ball;
  if (_ball.x() == std::numeric_limits<uint32_t>::max() &&
      _ball.y() == std::numeric_limits<uint32_t>::max()) {
    ball.x(xHat_[0](0));
    ball.y(xHat_[1](0));
  }

  // 状態観測器の状態方程式は一般に
  // x_hat_dot = (A - hC) * x_hat_ + h * y
  Eigen::Matrix<double, 2, 1> xHatDot;
  xHatDot = (A - h * C) * xHat_[0] + h * std::floor(ball.x() / quantLimitX_) * quantLimitX_;
  xHatDot *= passedTime;
  xHat_[0] += xHatDot;
  ball_.x(xHat_[0](0));
  ball_.vx(xHat_[0](1));

  // y軸方向についても同様に状態推定
  auto sinTheta = std::sin(theta);

  if (std::abs(friction_ * sinTheta) < std::abs(fricCoef_ * ball_.vy())) {
    // 摩擦力が飽和している領域
    auto fricCoefTuned = std::abs(friction_ * sinTheta / ball_.vy());

    A << 0, 1, 0, -(fricCoefTuned + airRegistance_);
    h << h1(fricCoefTuned), h2(fricCoefTuned);
  } else {
    // 摩擦が線形の領域。
    A << 0, 1, 0, -(fricCoef_ + airRegistance_);
    h << h1(fricCoef_), h2(fricCoef_);
  }

  xHatDot = (A - h * C) * xHat_[1] + h * std::floor(ball.y() / quantLimitY_) * quantLimitY_;
  xHatDot *= passedTime;
  xHat_[1] += xHatDot;
  ball_.y(xHat_[1](0));
  ball_.vy(xHat_[1](1));

  return ball_;
}
} // namespace observer
} // namespace filter
} // namespace ai
