#include "base.hpp"

namespace ai {
namespace controller {

velocity base::operator()(const model::robot& _robot, const position& _setpoint) {
  return update(_robot, _setpoint);
}

velocity base::operator()(const model::robot& _robot, const velocity& _setpoint) {
  return update(_robot, _setpoint);
}

} // namespace controller
} // namespace ai
