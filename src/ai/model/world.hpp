#ifndef AI_MODEL_WORLD_HPP_
#define AI_MODEL_WORLD_HPP_

#include <stdint.h>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include "ball.hpp"
#include "field.hpp"
#include "robot.hpp"

namespace ai {
namespace model {
class world {
  mutable std::shared_timed_mutex mutex_;

public:
  using RobotsList = std::unordered_map<uint32_t, model::robot>;
  world();
  world(model::field&& _field, model::ball&& _ball, RobotsList&& _robotsBlue,
        RobotsList&& _robotsYellow);
  world(const world& _others);
  world(world&& _others);

  world& operator=(const world& _others);
  world& operator=(world&& _others);

  model::ball ball() const;
  model::field field() const;
  RobotsList robotsBlue() const;
  RobotsList robotsYellow() const;

  template <class T>
  void field(T&& _field) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    field_ = std::forward<T>(_field);
  }

  template <class T>
  void ball(T&& _ball) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    ball_ = std::forward<T>(_ball);
  }

  template <class T>
  void robotsBlue(T&& _robotsBlue) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    robotsBlue_ = std::forward<T>(_robotsBlue);
  }

  template <class T>
  void robotsYellow(T&& _robotsYellow) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    robotsYellow_ = std::forward<T>(_robotsYellow);
  }

private:
  model::field field_;
  model::ball ball_;
  RobotsList robotsBlue_;
  RobotsList robotsYellow_;
};
}; // namespace model
} // namespace ai

#endif
