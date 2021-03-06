#include "ball.hpp"

namespace ai {
namespace model {
ball::ball() : x_(0), y_(0) {}

ball::ball(double _x, double _y) : x_(_x), y_(_y) {}

double ball::x() const {
  return x_;
}

double ball::y() const {
  return y_;
}

double ball::vx() const {
  return vx_;
}

double ball::vy() const {
  return vy_;
}

double ball::ax() const {
  return ax_;
}

double ball::ay() const {
  return ay_;
}

void ball::x(double _x) {
  x_ = _x;
}

void ball::y(double _y) {
  y_ = _y;
}

void ball::vx(double _vx) {
  vx_ = _vx;
}

void ball::vy(double _vy) {
  vy_ = _vy;
}

void ball::ax(double _ax) {
  ax_ = _ax;
}

void ball::ay(double _ay) {
  ay_ = _ay;
}
} // namespace model
} // namespace ai
