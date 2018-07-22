#ifndef AI_MODEL_COMMAND_HPP_
#define AI_MODEL_COMMAND_HPP_

#include <tuple>
#include <variant>
#include <stdint.h>

namespace ai {
namespace model {

class command {
public:
  enum class kickType { None, Straight, Tip };

  struct position {
    double x;
    double y;
    double theta;
  };

  struct velocity {
    double vx;
    double vy;
    double omega;
  };

  struct acceleration {
    double ax;
    double ay;
    double alpha;
  };

  using Setpoint = std::variant<position, velocity, acceleration>;
  using KickFlag = std::tuple<kickType, double>;

  explicit command(uint32_t _id);

  uint32_t id() const;
  int32_t dribble() const;
  KickFlag kick() const;
  const Setpoint& setpoint() const;

  void dribble(int32_t _dribble);
  void kick(const KickFlag& _kick);
  void pos(const position& _position);
  void vel(const velocity& _velocity);
  void accel(const acceleration& _accel);

private:
  uint32_t id_;
  int32_t dribble_;
  KickFlag kick_;
  Setpoint setpoint_;
};
} // namespace model
} // namespace ai

#endif
