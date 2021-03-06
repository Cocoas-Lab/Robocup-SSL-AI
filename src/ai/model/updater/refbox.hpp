#ifndef AI_SERVER_MODEL_UPDATER_REFBOX_H
#define AI_SERVER_MODEL_UPDATER_REFBOX_H

#include <shared_mutex>
#include <Eigen/Geometry>
#include "ai/model/refbox.hpp"

// 前方宣言
namespace ssl_protos {
namespace refbox {
class Referee;
}
} // namespace ssl_protos

namespace ai {
namespace model {
namespace updater {

/// @class   refbox
/// @brief   SSL RefBoxのRefereeパケットでRefBoxの情報を更新する
class refbox {
  mutable std::shared_timed_mutex mutex_;
  model::refbox refbox_;

  /// 変換行列
  Eigen::Affine3d affine_;

public:
  refbox();

  refbox(const refbox&) = delete;
  refbox& operator=(const refbox&) = delete;

  /// @brief          RefereeパケットでRefBoxの情報を更新する
  /// @param referee  SSL Referee BoxのRefereeパケット
  void update(const ssl_protos::refbox::Referee& _referee);

  /// @brief           updaterに変換行列を設定する
  /// @param matrix    変換行列
  void transformationMatrix(const Eigen::Affine3d& _matrix);

  /// @brief          値を取得する
  model::refbox value() const;
};

} // namespace updater
} // namespace model
} // namespace ai

#endif // AI_SERVER_MODEL_UPDATER_REFBOX_H
