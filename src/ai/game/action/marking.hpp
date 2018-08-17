#ifndef AI_GAME_ACTION_MARKING_HPP_
#define AI_GAME_ACTION_MARKING_HPP_

#include "base.hpp"
#include <stdint.h>

namespace ai {
namespace game {
namespace action {
class marking : public base {
public:
  enum markMode { PassCut, ShootBlock };

  using base::base;
  void mark(uint32_t _id);
  void mode(markMode _mode);
  model::command execute() override;

private:
  uint32_t enemyId_;
  markMode mode_;
};
} // namespace action
} // namespace game
} // namespace ai

#endif
