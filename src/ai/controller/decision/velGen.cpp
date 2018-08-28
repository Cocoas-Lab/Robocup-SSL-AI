#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <cmath>

#include "velGen.hpp"

namespace ai {
namespace controller {
namespace decision {

const double velGen::vMax_       = 8000.0;
const double velGen::aMax_       = 8000.0;
const double velGen::aMin_       = 5000.0;
const double velGen::reachSpeed_ = 1000.0;
const double velGen::kp_         = 1.0;

velGen::velGen(double _cycle) : cycle_(_cycle) {
  vTarget_ = {0.0, 0.0, 0.0};
}

double velGen::fromPos(const double _pos, double _target, const bool _stable) {
  if (std::abs(_pos) > 1000.0) {
    auto sign = boost::math::sign(-_pos);
    if (2 * std::pow(_target, 2) / (aMax_ * 2) > std::abs(_pos)) {
      _target += sign * aMax_ * cycle_;
      _target = std::clamp(_target, 0.0, vMax_);
    } else {
      _target += aMin_ * cycle_;
    }
    _target = std::clamp(_target, -vMax_, vMax_);
  } else {
    double k;
    if (_stable) {
      k = kp_;
    } else {
      k = kp_ * 10.0 * std::exp(-std::abs(_target) / 1000.0);
    }
    double requiredAccel = std::abs(_pos) * std::pow(k, 2);
    double state;

    // 必要な収束加速度が最大加速度を上回り,bangbang制御が必要か
    if (requiredAccel < aMax_) {
      // 普通のsliding_mode用state
      state = k * -_pos + _target;
    } else {
      // bangbang制御をするための特別なstate関数
      // 極度に非線形なため正か負かでstate切り替え
      int sgn   = boost::math::sign(-_pos);
      double pm = sgn * aMax_ / std::pow(k, 2);
      double vm = -sgn * aMax_ / k;
      state     = sgn * std::sqrt(sgn * (-_pos - pm)) + (_target - vm);
    }

    // sliding_mode
    if (std::abs(state) < aMax_ * cycle_) {
      _target = -k * -_pos; // stateが0になるように速度を保つ
    } else if (state < 0) {
      _target += aMin_ * cycle_; // stateが-なら加速
    } else {
      _target -= aMin_ * cycle_; // stateが+なら減速
    }
  }
  return _target;
}

double velGen::fromVel(const double _vel, double _target, const bool _stable) {
  double state = _target - _vel;
  // 制限加速度計算
  // 速度によって加速度が変化,初動でスリップしないように
  double optimizedAccel = _target * (aMax_ - aMin_) / reachSpeed_ + aMin_;
  optimizedAccel        = boost::algorithm::clamp(optimizedAccel, aMin_, aMax_);

  if (_stable) {
    optimizedAccel = aMin_ / 2.0;
  } else if (std::abs(_target) > std::abs(_vel) && _target * _vel > 0) {
    optimizedAccel = aMax_;
  } else {
    optimizedAccel = aMin_;
  }
  if (std::abs(state) < optimizedAccel * cycle_) {
    _target = _vel;
  } else if (state < 0) {
    _target += optimizedAccel * cycle_; // stateが-なら加速
  } else {
    _target -= optimizedAccel * cycle_; // stateが+なら減速
  }
  return _target;
}

velocity velGen::generate(const position& _pos, const bool _stable) {
  vTarget_.vx = fromPos(_pos.x, vTarget_.vx, _stable);
  vTarget_.vy = fromPos(_pos.y, vTarget_.vy, _stable);
  return vTarget_;
}

velocity velGen::generate(const velocity& _vel, const bool _stable) {
  vTarget_.vx = fromVel(_vel.vx, vTarget_.vx, _stable);
  vTarget_.vy = fromVel(_vel.vy, vTarget_.vy, _stable);
  return vTarget_;
}

} // namespace decision
} // namespace controller
} // namespace ai
