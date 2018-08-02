#ifndef AI_PLANNER_BASE_HPP_
#define AI_PLANNER_BASE_HPP_

#include <variant>
#include "ai/model/command.hpp"

namespace ai {
namespace planner {

using position     = model::command::position;
using velocity     = model::command::velocity;
using acceleration = model::command::acceleration;
using target       = std::variant<position, velocity>;

class base {
protected:
  position target_; // 目標(位置or速度)

public:
  base();
  virtual ~base();

  /// @brief  計算後の目標(位置or速度)を返す
  virtual position target();
};

} // namespace planner
} // namespace ai

#endif // AI_SERVER_PLANNER_BASE_H
