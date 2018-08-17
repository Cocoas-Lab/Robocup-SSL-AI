#ifndef AI_GAME_ACTION_MARKING_HPP_
#define AI_GAME_ACTION_MARKING_HPP_

#include "base.hpp"
#include <stdint.h>

namespace ai {
namespace game {
namespace action {
class marking : public base {
public:
  enum markMode {
    PassCut,   // ボールとロボットの間に入ってパスをカットする
    ShootBlock // ロボットとゴールの間に入ってシュートをブロックする
  };

  using base::base;

  /// @brief マーキング対象を指定する
  /// @param _id マーキング対象の敵ロボットID
  /// @return none
  void mark(uint32_t _id);

  /// @brief マーキングのモードを指定する
  /// @param _mode マーキングのモード
  /// @return none
  void mode(markMode _mode);

  /// @brief アクションの処理
  /// @return ロボットへの指令
  model::command execute() override;

private:
  uint32_t enemyId_; // マーキング対象のID
  markMode mode_;    // マーキングのモード
};
} // namespace action
} // namespace game
} // namespace ai

#endif
