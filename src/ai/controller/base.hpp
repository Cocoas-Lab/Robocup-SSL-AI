#ifndef AI_CONTROLLER_BASE_HPP_
#define AI_CONTROLLER_BASE_HPP_

#include <boost/variant.hpp>

#include "ai/model/command.hpp"
#include "ai/model/robot.hpp"

namespace ai {
namespace controller {

using position = model::command::position;
using velocity = model::command::velocity;

class base : public boost::static_visitor<velocity> {
public:
  base(const double _limit);
  virtual ~base() = default;

  velocity operator()(const model::robot& _robot, const position& _setpoint);
  velocity operator()(const model::robot& _robot, const velocity& _setpoint);

  double velocityLimit() const;
  virtual void velocityLimit(const double _limit);
  bool stable() const;
  virtual void stable(const bool _stable);

protected:
  double velocityLimit_;
  bool stable_;
  virtual velocity update(const model::robot& _robot, const position& _setpoint) = 0;
  virtual velocity update(const model::robot& _robot, const velocity& _setpoint) = 0;
};

} // namespace controller
} // namespace ai

#endif // AI_SERVER_CONTROLLER_BASE_H
