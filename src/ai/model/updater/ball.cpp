#include <algorithm>
#include <chrono>

#include "ai/util/math/affine.hpp"
#include "ai/util/time.hpp"
#include "ball.hpp"

namespace ai {
namespace model {
namespace updater {

ball::ball() : ball_{}, affine_{Eigen::Translation3d{.0, .0, .0}} {}

model::ball ball::value() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return ball_;
}

void ball::transformationMatrix(const Eigen::Affine3d& _matrix) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  affine_ = _matrix;
}

void ball::update(const ssl_protos::vision::DetectionFrame& _detection) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  // カメラID
  const auto& cameraId = _detection.camera_id();
  // キャプチャされた時間
  const auto capturedTime = util::TimePointType{util::toDuration(_detection.t_capture())};

  // 検出されたボールの中から, 最もconfidenceの高い値を選択候補に登録する
  // FIXME:
  // 現在の実装は, フィールドにボールが1つしかないと仮定している
  // 1つのカメラで複数のボールが検出された場合, 意図しないデータが選択される可能性がある
  const auto& balls    = _detection.balls();
  const auto candidate = std::max_element(balls.cbegin(), balls.cend(), [](auto& _a, auto& _b) {
    return _a.confidence() < _b.confidence();
  });
  if (candidate != balls.cend()) {
    rawBalls_[cameraId] = *candidate;
  } else {
    rawBalls_.erase(cameraId);
  }

  // 候補の中から, 最もconfidenceの高いボールを求める
  const auto reliable =
      std::max_element(rawBalls_.cbegin(), rawBalls_.cend(), [](auto& _a, auto& _b) {
        return std::get<1>(_a).confidence() < std::get<1>(_b).confidence();
      });

  if (reliable != rawBalls_.cend()) {
    // 選択された値のカメラIDとdetectionのカメラIDが一致していたらデータを更新する
    if (std::get<0>(*reliable) == cameraId) {
      const auto& value = std::get<1>(*reliable);
      reliableBall_     = util::math::transform(affine_, model::ball{value.x(), value.y()});

      if (onUpdatedFilter_) {
        // on_updated_filter_が設定されていたらFilterを通した値を使う
        ball_ = onUpdatedFilter_->update(*reliableBall_, capturedTime);
      } else if (!manualFilter_) {
        // Filterが登録されていない場合はそのままの値を使う
        ball_.x(reliableBall_->x());
        ball_.y(reliableBall_->y());
      }
    }
  } else {
    // 現時点ではボールが存在しない場合を想定していないので何もしない
    reliableBall_ = std::nullopt;
  }
}

void ball::clearFilter() {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  onUpdatedFilter_.reset();
  manualFilter_.reset();
}

} // namespace updater
} // namespace model
} // namespace ai
