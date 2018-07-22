#include <cmath>

#include "field.hpp"
#include "ssl-protos/vision/geometry.pb.h"

namespace ai {
namespace model {
namespace updater {

field::field() : field_{} {}

void field::update(const ssl_protos::vision::GeometryData& _geometry) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  const auto& f = _geometry.field();

  field_.length(f.field_length());
  field_.width(f.field_width());
  field_.goalWidth(f.goal_width());

  for (const auto& arc : f.field_arcs()) {
    if (arc.name() == "CenterCircle") {
      field_.centerRadius(arc.radius());
    }
  }
}

model::field field::value() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return field_;
}

} // namespace updater
} // namespace model
} // namespace ai
