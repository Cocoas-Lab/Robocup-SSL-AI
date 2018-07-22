#ifndef AI_MODEL_ROBOT_HPP_
#define AI_MODEL_ROBOT_HPP_

#include <stdint.h>

namespace ai {
namespace model {
class robot {
  uint32_t id_;
  double x_;
  double y_;
  double vx_;
  double vy_;
  double ax_;
  double ay_;
  double theta_;
  double omega_;

public:
  robot(uint32_t _id = 0, double _x = 0, double _y = 0, double _theta = 0);
  robot(const robot&) = default;

  uint32_t id() const;
  double x() const;
  double y() const;
  double vx() const;
  double vy() const;
  double ax() const;
  double ay() const;
  double theta() const;
  double omega() const;

  void x(double _x);
  void y(double _y);
  void vx(double _vx);
  void vy(double _vy);
  void ax(double _ax);
  void ay(double _ay);
  void theta(double _theta);
  void omega(double _omega);
};
} // namespace model
} // namespace ai

#endif
