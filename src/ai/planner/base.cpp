#include "base.hpp"

namespace ai {
namespace planner {

base::base() {}

base::~base() {}

position base::target() {
  return target_;
}

} // namespace planner
} // namespace ai
