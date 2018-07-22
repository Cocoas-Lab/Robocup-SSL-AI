#ifndef AI_MODEL_UPDATER_ROBOT_HPP_
#define AI_MODEL_UPDATER_ROBOT_HPP_

#include <functional>
#include <memory>
#include <shared_mutex>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <stdint.h>

#include <Eigen/Geometry>

#include "ai/filter/base.hpp"
#include "ai/model/robot.hpp"
#include "ai/model/teamColor.hpp"
#include "ssl-protos/vision/detection.pb.h"

namespace ai {
namespace model {
namespace updater {

/// @class   robot
/// @brief   SSL-VisionのDetectionパケットでロボットの情報を更新する
template <model::teamColor Color>
class robot {
  /// KeyがID, Valueがロボットのハッシュテーブルの型
  using RobotsListType = std::unordered_map<uint32_t, model::robot>;

  /// 生データの型
  using RawDataType = ssl_protos::vision::DetectionRobot;
  /// 生データが格納されている配列の型
  using RawDataArrayType = google::protobuf::RepeatedPtrField<RawDataType>;
  /// ロボットの生データを取得するFrameのメンバ関数へのポインタの型
  using SourceFunctionPointerType =
      const RawDataArrayType& (ssl_protos::vision::DetectionFrame::*)() const;

  /// 更新タイミングがon_updatedなFilterの型
  using OnUpdatedFilterType = filter::base<model::robot, filter::timing::OnUpdated>;
  /// 更新タイミングがmanualなFilterの型
  using ManualFilterType = filter::base<model::robot, filter::timing::Manual>;

public:
  robot();
  robot(const robot&) = delete;
  robot& operator=(const robot&) = delete;

  /// @brief           Detectionパケットを処理し, ボールの情報を更新する
  /// @param detection SSL-VisionのDetectionパケット
  void update(const ssl_protos::vision::DetectionFrame& _detection);

  /// @brief           値を取得する
  RobotsListType value() const;

  /// @brief           updaterに変換行列を設定する
  /// @param matrix    変換行列
  void transformationMatrix(const Eigen::Affine3d& _matrix);

  /// @brief           設定されたFilterを解除する
  /// @param id        Filterを解除するロボットのID
  void clearFilter(uint32_t _id);

  /// @brief           設定された全てのFilterを解除する
  void clearAllFilters();

  /// @brief           設定されたデフォルトのFilterを解除する
  ///
  /// あくまでfilter_initializer_を空にするだけなので,
  /// 既にfilter_initializer_によって初期化されたものを解除したい場合は
  /// clear_filter()などを呼ぶ必要がある
  void clearDefaultFilter();

  /// @brief           更新タイミングがon_updatedなFilterを設定する
  /// @param id        Filterを設定するロボットのID
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<std::enable_if_t<std::is_base_of<OnUpdatedFilterType, Filter>::value, Filter>>
  setFilter(uint32_t _id, Args&&... _args) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    manualFilters_.erase(_id);
    auto p                 = std::make_shared<Filter>(std::forward<Args>(_args)...);
    onUpdatedFilters_[_id] = p;
    return p;
  }

  /// @brief           更新タイミングがmanualなFilterを設定する
  /// @param id        Filterを設定するロボットのID
  /// @param args      Filterの引数
  /// @return          初期化されたFilterへのポインタ
  template <class Filter, class... Args>
  std::weak_ptr<std::enable_if_t<std::is_base_of<ManualFilterType, Filter>::value, Filter>>
  setFilter(uint32_t _id, Args&&... _args) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    onUpdatedFilters_.erase(_id);
    auto p = std::make_shared<Filter>(
        // 最新の値を取得する関数オブジェクト
        [_id, this]() -> std::optional<model::robot> {
          std::shared_lock<std::shared_timed_mutex> lock(mutex_);
          // 選ばれた観測データの中に対象のロボットの値があればそれを, なければnulloptを返す
          if (reliableRobots_.count(_id)) {
            return reliableRobots_.at(_id);
          } else {
            return std::nullopt;
          }
        },
        // 値を更新する関数オブジェクト
        [_id, this](std::optional<model::robot> _value) {
          std::unique_lock<std::shared_timed_mutex> lock(mutex_);
          if (_value) {
            // valueが値を持っていた場合はその値で更新
            robots_[_id] = *_value;
          } else {
            // valueが値を持っていなかった場合はリストから要素を削除する
            robots_.erase(_id);
          }
        },
        // 残りの引数
        std::forward<Args>(_args)...);
    manualFilters_[_id] = p;
    return p;
  }

  /// @brief           デフォルトのFilterを設定する
  /// @param args      Filterの引数
  ///
  /// set_filter()でFilterが設定されていないロボットが使うFilterを設定する
  /// ここで設定できるものは更新タイミングがon_updatedなFilterのみとする
  template <class Filter, class... Args>
  auto setDefaultFilter(Args... _args)
      -> std::enable_if_t<std::is_base_of<OnUpdatedFilterType, Filter>::value> {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    filterInitializer_ = [_args...] { return std::make_shared<Filter>(_args...); };
  }

private:
  mutable std::shared_timed_mutex mutex_;

  /// ロボットの生データを取得するFrameのメンバ関数へのポインタ
  /// このポインタを各チームカラーに対して特殊化することで, 同じ更新処理を使えるようにしている
  static const SourceFunctionPointerType src_;

  /// 最終的な値
  RobotsListType robots_;

  /// 各カメラで検出されたロボットの生データ (KeyはカメラID)
  std::unordered_map<uint32_t, RawDataArrayType> rawRobots_;
  /// 検出された中から選ばれた, 各IDの最も確かとされる値のリスト
  RobotsListType reliableRobots_;

  /// 更新タイミングがon_updatedなFilter
  std::unordered_map<uint32_t, std::shared_ptr<OnUpdatedFilterType>> onUpdatedFilters_;
  /// 更新タイミングがmanualなFilter
  std::unordered_map<uint32_t, std::shared_ptr<ManualFilterType>> manualFilters_;
  /// Filterを初期化するための関数オブジェクト
  std::function<std::shared_ptr<OnUpdatedFilterType>()> filterInitializer_;

  /// 変換行列
  Eigen::Affine3d affine_;
};

} // namespace updater
} // namespace model
} // namespace ai

#endif // AI_SERVER_MODEL_UPDATER_ROBOT_H
