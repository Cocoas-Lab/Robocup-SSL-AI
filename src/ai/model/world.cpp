#include <mutex>

#include "world.hpp"

namespace ai {
namespace model {
world::world() : field_{}, ball_{}, robotsBlue_{}, robotsYellow_{} {}

world::world(model::field&& _field, model::ball&& _ball, RobotsList&& _robotsBlue,
             RobotsList&& _robotsYellow)
    : field_(std::move(_field)),
      ball_(std::move(_ball)),
      robotsBlue_(std::move(_robotsBlue)),
      robotsYellow_(std::move(_robotsYellow)) {}

world::world(const world& _others)
    : field_(_others.field()),
      ball_(_others.ball()),
      robotsBlue_(_others.robotsBlue()),
      robotsYellow_(_others.robotsYellow()) {}

world::world(world&& _others) {
  std::unique_lock<std::shared_timed_mutex> lock(_others.mutex_);
  std::swap(field_, _others.field_);
  std::swap(ball_, _others.ball_);
  std::swap(robotsBlue_, _others.robotsBlue_);
  std::swap(robotsYellow_, _others.robotsYellow_);
}

world& world::operator=(const world& _others) {
  std::unique_lock<std::shared_timed_mutex> lock1(mutex_, std::defer_lock);
  std::shared_lock<std::shared_timed_mutex> lock2(_others.mutex_, std::defer_lock);
  std::lock(lock1, lock2);
  field_        = _others.field_;
  ball_         = _others.ball_;
  robotsBlue_   = _others.robotsBlue_;
  robotsYellow_ = _others.robotsYellow_;
  return *this;
}

world& world::operator=(world&& _others) {
  std::unique_lock<std::shared_timed_mutex> lock1(mutex_, std::defer_lock);
  std::unique_lock<std::shared_timed_mutex> lock2(_others.mutex_, std::defer_lock);
  std::lock(lock1, lock2);
  std::swap(field_, _others.field_);
  std::swap(ball_, _others.ball_);
  std::swap(robotsBlue_, _others.robotsBlue_);
  std::swap(robotsYellow_, _others.robotsYellow_);
  return *this;
}

model::field world::field() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return field_;
}

model::ball world::ball() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return ball_;
}

world::RobotsList world::robotsBlue() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return robotsBlue_;
}

world::RobotsList world::robotsYellow() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return robotsYellow_;
}
} // namespace model
} // namespace ai
