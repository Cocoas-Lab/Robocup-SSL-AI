#ifndef AI_MODEL_UPDATER_BALL_HPP_
#define AI_MODEL_UPDATER_BALL_HPP_

#include <memory>
#include <optional>
#include <shared_mutex>
#include <type_traits>
#include <unordered_map>

#include <Eigen/Geometry>

#include "ai/filter/base.hpp"
#include "ai/model/ball.hpp"
#include "ssl-protos/vision/detection.pb.h"

namespace ai {
namespace model {
namespace updater {

/// @class   ball
/// @brief   SSL-VisionのDetectionパケットでボールの情報を更新する
class ball {
  /// 更新タイミングがon_updatedなFilterの型
  using OnUpdatedFilterType = filter::base<model::ball, filter::timing::OnUpdated>;
  /// 更新タイミングがmanualなFilterの型
  using ManualFilterType = filter::base<model::ball, filter::timing::Manual>;

public:
  ball();

  ball(const ball&) = delete;
  ball& operator=(const ball&) = delete;

  /// @brief           Detectionパケットを処理し, ボールの情報を更新する
  /// @param detection SSL-VisionのDetectionパケット
  void update(const ssl_protos::vision::DetectionFrame& _detection);

  /// @brief           値を取得する
  model::ball value() const;

  /// @brief           updaterに変換行列を設定する
  /// @param matrix    変換行列
  void transformationMatrix(const Eigen::Affine3d& _matrix);

  /// @brief           設定されたFilterを解除する
  void clearFilter();

  /// @brief           更新タイミングがon_updatedなFilterを設定する
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<std::enable_if_t<std::is_base_of<OnUpdatedFilterType, Filter>::value, Filter>>
  setFilter(Args&&... _args) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    manualFilter_.reset();
    auto p           = std::make_shared<Filter>(std::forward<Args>(_args)...);
    onUpdatedFilter_ = p;
    return p;
  }

  /// @brief           更新タイミングがmanualなFilterを設定する
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<std::enable_if_t<std::is_base_of<ManualFilterType, Filter>::value, Filter>>
  setFilter(Args&&... _args) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    onUpdatedFilter_.reset();
    auto p = std::make_shared<Filter>(
        // 最新の値を取得する関数オブジェクト
        [this] {
          std::shared_lock<std::shared_timed_mutex> lock(mutex_);
          return reliableBall_;
        },
        // 値を更新する関数オブジェクト
        [this](std::optional<model::ball> _value) {
          // 現時点ではボールが存在しない場合を想定していないので,
          // 値を持っていた場合のみ値の更新を行う
          if (_value) {
            std::unique_lock<std::shared_timed_mutex> lock(mutex_);
            ball_ = *_value;
          }
        },
        // 残りの引数
        std::forward<Args>(_args)...);
    manualFilter_ = p;
    return p;
  }

private:
  mutable std::shared_timed_mutex mutex_;

  /// 最終的な値
  model::ball ball_;

  /// 各カメラで検出されたボールの生データ
  std::unordered_map<uint32_t, ssl_protos::vision::DetectionBall> rawBalls_;
  /// 検出された中から選ばれた, 最も確かとされる値
  std::optional<model::ball> reliableBall_;

  /// 更新タイミングがon_updatedなFilter
  std::shared_ptr<OnUpdatedFilterType> onUpdatedFilter_;
  /// 更新タイミングがmanualなFilter
  std::shared_ptr<ManualFilterType> manualFilter_;

  /// 変換行列
  Eigen::Affine3d affine_;
};

} // namespace updater
} // namespace model
} // namespace ai

#endif // AI_SERVER_MODEL_UPDATER_BALL_H
