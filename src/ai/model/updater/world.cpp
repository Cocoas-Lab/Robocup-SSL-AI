#include "ai/util/math/affine.hpp"
#include "world.hpp"
#include "ssl-protos/vision/wrapper.pb.h"

namespace ai {
namespace model {
namespace updater {

void world::update(const ssl_protos::vision::WrapperPacket& _packet) {
  if (_packet.has_detection()) {
    const auto& detection = _packet.detection();

    // 無効化されたカメラは無視する
    if (std::find(disabledCamera_.cbegin(), disabledCamera_.cend(), detection.camera_id()) !=
        disabledCamera_.cend()) {
      return;
    }

    ball_.update(detection);
    robotsBlue_.update(detection);
    robotsYellow_.update(detection);
  }

  if (_packet.has_geometry()) {
    const auto& geometry = _packet.geometry();
    field_.update(geometry);
  }
}

model::world world::value() const {
  return {field_.value(), ball_.value(), robotsBlue_.value(), robotsYellow_.value()};
}

void world::transformationMatrix(const Eigen::Affine3d& _matrix) {
  ball_.transformationMatrix(_matrix);
  robotsBlue_.transformationMatrix(_matrix);
  robotsYellow_.transformationMatrix(_matrix);
}

void world::transformationMatrix(double _x, double _y, double _theta) {
  transformationMatrix(util::math::makeTransformationMatrix(_x, _y, _theta));
}

void world::disableCamera(uint32_t _id) {
  // idが登録されていなかったら追加する
  if (std::find(disabledCamera_.cbegin(), disabledCamera_.cend(), _id) ==
      disabledCamera_.cend()) {
    disabledCamera_.push_back(_id);
  }
}

void world::enableCamera(uint32_t _id) {
  // idが登録されていたら解除する
  auto it = std::find(disabledCamera_.begin(), disabledCamera_.end(), _id);
  if (it != disabledCamera_.end()) {
    disabledCamera_.erase(it);
  }
}

bool world::isCameraEnabled(uint32_t _id) const {
  return std::find(disabledCamera_.cbegin(), disabledCamera_.cend(), _id) ==
         disabledCamera_.cend();
}

field& world::fieldUpdater() {
  return field_;
}

ball& world::ballUpdater() {
  return ball_;
}

robot<model::teamColor::Blue>& world::robotsBlueUpdater() {
  return robotsBlue_;
}

robot<model::teamColor::Yellow>& world::robotsYellowUpdater() {
  return robotsYellow_;
}

} // namespace updater
} // namespace model
} // namespace ai
