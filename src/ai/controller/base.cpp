#include "base.hpp"

namespace ai {
namespace controller {
base::base(const double _limit) : velocityLimit_(_limit), stable_(false) {}

velocity base::operator()(const model::robot& _robot, const position& _setpoint) {
  return update(_robot, _setpoint);
}

velocity base::operator()(const model::robot& _robot, const velocity& _setpoint) {
  return update(_robot, _setpoint);
}

double base::velocityLimit() const {
  return velocityLimit_;
}

void base::velocityLimit(const double _limit) {
  velocityLimit_ = _limit;
}

bool base::stable() const {
  return stable_;
}

void base::stable(const bool _stable) {
  stable_ = _stable;
}
} // namespace controller
} // namespace ai
