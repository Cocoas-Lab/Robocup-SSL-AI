#ifndef AI_GAME_ACTION_BASE_H
#define AI_GAME_ACTION_BASE_H

#include <memory>
#include <stdint.h>
#include "ai/model/command.hpp"
#include "ai/model/world.hpp"
#include "ai/planner/rrt.hpp"

namespace ai {
namespace game {
namespace action {

class base {
public:
  /// @param _world            WorldModelの参照
  /// @param _isYellow        チームカラーは黄色か
  /// @param _id               操作するロボットのID
  base(const model::world& _world, bool _isYellow, uint32_t _id);

  virtual ~base() = default;

  /// @brief                  割り当てられているidを取得する
  uint32_t id() const;

  /// @brief                  呼び出されたループでのロボットの命令を取得する
  virtual model::command execute() = 0;

  /// @brief                  Actionが完了したか
  bool finished() const;

protected:
  const model::world& world_;
  bool isYellow_;
  bool finished_;
  uint32_t id_;
  std::shared_ptr<planner::rrt> rrt_;
};

} // namespace action
} // namespace game
} // namespace ai

#endif // AI_GAME_ACTION_BASE_H
