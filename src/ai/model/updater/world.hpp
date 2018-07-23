#ifndef AI_MODEL_UPDATER_WORLD_HPP_
#define AI_MODEL_UPDATER_WORLD_HPP_

#include <vector>
#include <Eigen/Geometry>
#include <stdint.h>

#include "ai/model/world.hpp"
#include "ball.hpp"
#include "field.hpp"
#include "robot.hpp"

// 前方宣言
namespace ssl_protos {
namespace vision {
class WrapperPacket;
}
} // namespace ssl_protos

namespace ai {
namespace model {
namespace updater {

class world {
  /// フィールドのupdater
  field field_;
  /// ボールのupdater
  ball ball_;
  /// 青ロボットのupdater
  robot<model::teamColor::Blue> robotsBlue_;
  /// 黄ロボットのupdater
  robot<model::teamColor::Yellow> robotsYellow_;

  /// 無効化されたカメラID
  std::vector<uint32_t> disabledCamera_;

public:
  world()             = default;
  world(const world&) = delete;
  world& operator=(const world&) = delete;

  /// @brief                  内部の状態を更新する
  /// @param packet           SSL-Visionのパース済みパケット
  void update(const ssl_protos::vision::WrapperPacket& _packet);

  /// @brief           値を取得する
  model::world value() const;

  /// @brief           updaterに変換行列を設定する
  /// @param matrix    変換行列
  void transformationMatrix(const Eigen::Affine3d& _matrix);
  /// @brief           updaterに変換行列を設定する
  /// @param x         x軸方向に平行移動する量
  /// @param y         y軸方向に平行移動する量
  /// @param theta     z軸を中心に回転する量
  void transformationMatrix(double _x, double _y, double _theta);

  /// @brief                  指定したIDのカメラで更新しないようにする
  /// @param id               カメラID
  void disableCamera(uint32_t _id);

  /// @brief                  無効化したカメラを有効にする
  /// @param id               カメラID
  void enableCamera(uint32_t _id);

  /// @brief                  カメラが有効か調べる
  /// @param id               カメラID
  bool isCameraEnabled(uint32_t _id) const;

  /// @brief           フィールドのupdaterを取得する
  field& fieldUpdater();
  /// @brief           ボールのupdaterを取得する
  ball& ballUpdater();
  /// @brief           青ロボットのupdaterを取得する
  robot<model::teamColor::Blue>& robotsBlueUpdater();
  /// @brief           黄ロボットのupdaterを取得する
  robot<model::teamColor::Yellow>& robotsYellowUpdater();
};

} // namespace updater
} // namespace model
} // namespace ai

#endif // AI_SERVER_MODEL_UPDATER_WORLD_H
