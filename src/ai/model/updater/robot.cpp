#include <algorithm>

#include "ai/util/math/affine.hpp"
#include "ai/util/time.hpp"
#include "robot.hpp"

namespace ai {
namespace model {
namespace updater {

// robot<Color>::src_の特殊化
// 青チームのロボットはssl_protos::vision::Frame::robots_blue()で取得する
template <>
const robot<model::teamColor::Blue>::SourceFunctionPointerType
    robot<model::teamColor::Blue>::src_ = &ssl_protos::vision::DetectionFrame::robots_blue;

// 黄チームのロボットはssl_protos::vision::Frame::robots_yellow()で取得する
template <>
const robot<model::teamColor::Yellow>::SourceFunctionPointerType
    robot<model::teamColor::Yellow>::src_ = &ssl_protos::vision::DetectionFrame::robots_yellow;

template <model::teamColor Color>
robot<Color>::robot() : affine_{Eigen::Translation3d{.0, .0, .0}} {}

template <model::teamColor Color>
void robot<Color>::update(const ssl_protos::vision::DetectionFrame& _detection) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  // カメラID
  const auto cameraId = _detection.camera_id();
  // キャプチャされた時間
  const auto capturedTime = util::TimePointType{util::toDuration(_detection.t_capture())};

  // 保持している生データを更新する
  rawRobots_[cameraId] = (_detection.*src_)();

  // カメラIDをKeyに, 各カメラで検出されたロボットの情報を保持しているraw_robots_から
  // | Key(cam_id) | Value({Robot})           |
  // | ----------- | ------------------------ |
  // |           0 | {Robot(ID0), Robot(ID1)} |
  // |           1 | {Robot(ID1), Robot(ID2)} |
  // ロボットIDをKeyに,
  // 1つ以上のロボットの情報をカメラIDとともに保持するハッシュテーブルを作る
  // | Key(robo_id)  | Value({cam_id, Robot})                       |
  // | ------------- | -------------------------------------------- |
  // |             0 | {{cam_id, Robot(ID0)}}                       |
  // |             1 | {{cam_id, Robot(ID1)}, {cam_id, Robot(ID1)}} |
  // |             2 | {{cam_id, Robot(ID2)}}                       |
  const auto candidates = [this] {
    using RobotWithCameraId = std::tuple<uint32_t, RawDataArrayType::const_iterator>;
    std::unordered_multimap<uint32_t, RobotWithCameraId> table{};
    for (auto it1 = rawRobots_.cbegin(); it1 != rawRobots_.cend(); ++it1) {
      const auto& robotsList = it1->second;
      for (auto it2 = robotsList.cbegin(); it2 != robotsList.cend(); ++it2) {
        table.emplace(it2->robot_id(), std::forward_as_tuple(it1->first, it2));
      }
    }
    return table;
  }();

  // 作ったハッシュテーブルから, 各IDの最もconfidenceの高い要素を選択して値の更新を行う
  RobotsListType reliables{};
  for (auto it = candidates.cbegin(); it != candidates.cend();) {
    const auto robotId = it->first;

    // IDがrobot_idのロボットの中で, 最もconfidenceの高い値を選択する
    const auto range    = candidates.equal_range(robotId);
    const auto reliable = std::max_element(range.first, range.second, [](auto& _a, auto& _b) {
      return std::get<1>(_a.second)->confidence() < std::get<1>(_b.second)->confidence();
    });

    // その値が検出されたカメラIDとdetectionのカメラIDを比較
    if (std::get<0>(reliable->second) == cameraId) {
      // 一致していたら値の更新を行う
      // (現在のカメラで新たに検出された or
      // 現在のカメラで検出された値のほうがconfidenceが高かった)
      const auto value   = util::math::transform(affine_, [reliable] {
        const auto r = std::get<1>(reliable->second);
        return model::robot{r->robot_id(), r->x(), r->y(), r->orientation()};
      }());
      reliables[robotId] = value;

      // 2つのFilterが設定されておらず, かつfilter_initializer_が設定されていたら
      // filter_initializer_でFilterを初期化する
      if (filterInitializer_ && !onUpdatedFilters_.count(robotId) &&
          !manualFilters_.count(robotId)) {
        onUpdatedFilters_[robotId] = filterInitializer_();
      }

      if (onUpdatedFilters_.count(robotId)) {
        // on_updated_filter_が設定されていたらFilterを通した値を使う
        robots_[robotId] = onUpdatedFilters_.at(robotId)->update(value, capturedTime);
      } else if (!manualFilters_.count(robotId)) {
        // Filterが登録されていない場合はそのままの値を使う
        robots_[robotId] = value;
      }
    } else {
      // カメラIDが一致しないときは前の値を引き継ぐ
      // (現在のカメラで検出されたがconfidenceが低かった or 現在のカメラで検出されなかった)
      reliables[robotId] = reliableRobots_.at(robotId);
    }

    // イテレータを次のロボットIDの位置まで進める
    it = range.second;
  }
  reliableRobots_ = std::move(reliables);

  // 最終的なデータのリストから, フィールド全体で検出されなかったIDを取り除く
  // ただし, manual_filterが設定されている場合は例外とする
  for (auto it = robots_.begin(); it != robots_.end();) {
    const auto id = it->first;
    if (reliableRobots_.count(id) || manualFilters_.count(id)) {
      ++it;
    } else {
      it = robots_.erase(it);
    }
  }
}

template <model::teamColor Color>
typename robot<Color>::RobotsListType robot<Color>::value() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return robots_;
}

template <model::teamColor Color>
void robot<Color>::transformationMatrix(const Eigen::Affine3d& matrix) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  affine_ = matrix;
}

template <model::teamColor Color>
void robot<Color>::clearFilter(uint32_t _id) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  onUpdatedFilters_.erase(_id);
  manualFilters_.erase(_id);
}

template <model::teamColor Color>
void robot<Color>::clearAllFilters() {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  onUpdatedFilters_.clear();
  manualFilters_.clear();
}

template <model::teamColor Color>
void robot<Color>::clearDefaultFilter() {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  filterInitializer_ = decltype(filterInitializer_){};
}

// 必要なチームカラーで明示的なtemplateのインスタンス化を行う
// これにより, class templateを用いているが実装をソースに分離することができる
// http://en.cppreference.com/w/cpp/language/class_template#Explicit_instantiation
template class robot<model::teamColor::Blue>;
template class robot<model::teamColor::Yellow>;

} // namespace updater
} // namespace model
} // namespace ai
