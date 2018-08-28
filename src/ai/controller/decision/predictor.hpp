#ifndef AI_CONTROLLER_DECISION_PREDICTOR_HPP_
#define AI_CONTROLLER_DECISION_PREDICTOR_HPP_

#include <Eigen/Core>

#include "ai/model/command.hpp"
#include "ai/model/robot.hpp"

namespace ai {
namespace controller {
namespace decision {

using position     = model::command::position;
using velocity     = model::command::velocity;
using acceleration = model::command::acceleration;

/// @class  smith_predictor
/// @brief  無駄時間補間,visionからのデータが7フレーム遅れるとし,その分を補間
class predictor {
private:
  double cycle_; // 制御周期
  // 二次遅れモデルパラメータ
  double zeta_;
  double omega_;
  Eigen::Vector3d u_[7]; // 7フレーム分の制御入力

public:
  /// @brief  コンストラクタ
  /// @param  cycle 制御周期
  /// @param  zeta  ロボット二次遅れモデルのパラメータζ
  /// @param  omega ロボット二次遅れモデルのパラメータω
  predictor(const double _cycle, const double _zeta, const double _omega);

  /// @brief  現在状態の推定
  /// @param  robot ロボット
  /// @param  u (前回)制御入力
  Eigen::Matrix3d interpolate(const model::robot& _robot, const Eigen::Vector3d& _u);
};

} // namespace decision
} // namespace controller
} // namespace ai

#endif // AI_SERVER_CONTROLLER_DETAIL_SMITH_PREDICTOR_H
