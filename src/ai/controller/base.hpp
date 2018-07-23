#ifndef AI_SERVER_CONTROLLER_BASE_H
#define AI_SERVER_CONTROLLER_BASE_H

#include <boost/variant.hpp>

#include "ai/model/command.hpp"
#include "ai/model/robot.hpp"

namespace ai {
namespace controller {

using position = model::command::position;
using velocity = model::command::velocity;

class base : public boost::static_visitor<velocity> {
public:
  virtual ~base() = default;

  velocity operator()(const model::robot& _robot, const position& _setpoint);
  velocity operator()(const model::robot& _robot, const velocity& _setpoint);

protected:
  virtual velocity update(const model::robot& _robot, const position& _setpoint) = 0;
  virtual velocity update(const model::robot& _robot, const velocity& _setpoint) = 0;
};

} // namespace controller
} // namespace ai

#endif // AI_SERVER_CONTROLLER_BASE_H
