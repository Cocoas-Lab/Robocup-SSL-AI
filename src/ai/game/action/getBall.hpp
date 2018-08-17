#ifndef AI_GAME_ACTION_GET_BALL_HPP_
#define AI_GAME_ACTION_GET_BALL_HPP_

#include "base.hpp"
#include "ai/model/command.hpp"

namespace ai {
namespace game {
namespace action {
using position = model::command::position;
using velocity = model::command::velocity;
class getBall : public base {
  position target_;

public:
  using base::base;
  void setTarget(const position _target);
  model::command execute() override;
};
} // namespace action
} // namespace game
} // namespace ai

#endif
