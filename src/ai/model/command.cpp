#include "command.hpp"

namespace ai {
namespace model {

command::command(uint32_t _id)
    : id_(_id),
      dribble_(0),
      kick_(command::kickType::None, 0.0),
      setpoint_(command::velocity{0.0, 0.0, 0.0}) {}

uint32_t command::id() const {
  return id_;
}

int32_t command::dribble() const {
  return dribble_;
}

command::KickFlag command::kick() const {
  return kick_;
}

const command::Setpoint& command::setpoint() const {
  return setpoint_;
}

void command::dribble(int32_t _dribble) {
  dribble_ = _dribble;
}

void command::kick(const command::KickFlag& _kick) {
  kick_ = _kick;
}

void command::pos(const command::position& _position) {
  setpoint_ = _position;
}

void command::vel(const command::velocity& _velocity) {
  setpoint_ = _velocity;
}

} // namespace model
} // namespace ai
