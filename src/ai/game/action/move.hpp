#ifndef AI_GAME_ACTION_MOVE_H
#define AI_GAME_ACTION_MOVE_H

#include "base.hpp"
#include "ai/model/command.hpp"
#include <memory>

namespace ai {
namespace game {
namespace action {

class move : public base {
public:
  using base::base;

  void moveTo(double _x, double _y, double _theta = 0.0);

  model::command execute() override;

private:
  double x_;
  double y_;
  double theta_;
};
} // namespace action
} // namespace game
} // namespace ai

#endif
