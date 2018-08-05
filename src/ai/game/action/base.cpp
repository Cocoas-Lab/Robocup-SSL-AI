#include "base.hpp"

namespace ai {
namespace game {
namespace action {

base::base(const model::world& _world, bool _isYellow, uint32_t _id)
    : world_(_world), isYellow_(_isYellow), finished_(false), id_(_id), rrt_(std::make_shared<planner::rrt>(_world)) {}

uint32_t base::id() const {
  return id_;
}

bool base::finished() const{
	return finished_;
}
} // namespace action
} // namespace game
} // namespace ai_serve
