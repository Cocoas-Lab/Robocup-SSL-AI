#ifndef AI_MODEL_BALL_HPP_
#define AI_MODEL_BALL_HPP_

namespace ai {
namespace model {
class ball {
  double x_;
  double y_;
  double vx_;
  double vy_;
  double ax_;
  double ay_;

public:
  ball();
  ball(double _x, double _y);
  ball(const ball&) = default;

  double x();
  double y();
  double vx();
  double vy();
  double ax();
  double ay();

  void x(double _x);
  void y(double _y);
  void vx(double _vx);
  void vy(double _vy);
  void ax(double _ax);
  void ay(double _ay);
};
} // namespace model
} // namespace ai

#endif
