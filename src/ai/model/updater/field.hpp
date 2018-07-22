#ifndef AI_MODEL_UPDATER_FIELD_HPP_
#define AI_MODEL_UPDATER_FIELD_HPP_

#include <shared_mutex>
#include "ai/model/field.hpp"

// 前方宣言
namespace ssl_protos {
namespace vision {
class GeometryData;
}
} // namespace ssl_protos

namespace ai {
namespace model {
namespace updater {

/// @class   field
/// @brief   SSL-VisionのGeometryパケットでフィールドの情報を更新する
class field {
  mutable std::shared_timed_mutex mutex_;
  model::field field_;

public:
  field();

  field(const field&) = delete;
  field& operator=(const field&) = delete;

  /// @brief          Geometryパケットを処理し, フィールドの情報を更新する
  /// @param geometry SSL-VisionのGeometryパケット
  void update(const ssl_protos::vision::GeometryData& _geometry);

  /// @brief          値を取得する
  model::field value() const;
};

} // namespace updater
} // namespace model
} // namespace ai

#endif // AI_SERVER_MODEL_UPDATER_FIELD_H
