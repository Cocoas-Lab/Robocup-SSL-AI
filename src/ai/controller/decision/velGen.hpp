#ifndef AI_CONTROLLER_DECISION_VEL_GEN_HPP_
#define AI_CONTROLLER_DECISION_VEL_GEN_HPP_

#include "ai/model/command.hpp"

namespace ai {
namespace controller {
namespace decision {

using position = model::command::position;
using velocity = model::command::velocity;

/// @class  velGen
/// @brief  速度生成器
class velGen {
private:
  velocity vTarget_;               // 目標指令速度
  double cycle_;                   // 周期
  static const double vMax_;       //
  static const double aMax_;       // 最大加速度
  static const double aMin_;       // 最小加速度
  static const double reachSpeed_; // 最大加速度に達するときの速度
  static const double kp_;         // 収束速度パラメータ

  double fromPos(const double _pos, double _target, const bool _stable);
  double fromVel(const double _vel, double _target, const bool _stable);

public:
  /// @brief  コンストラクタ
  /// @param  cycle 制御周期
  velGen(double _cycle);

  /// @brief  位置制御計算関数
  /// @param  delta_p  位置偏差(現在位置-目標位置)
  /// @param  stable   安定制御用(true->安定,false->通常)
  velocity generate(const position& _pos, const bool _stable);

  /// @brief  速度制御計算関数
  /// @param  target   目標速度
  /// @param  stable   安定制御用(true->安定,false->通常)
  velocity generate(const velocity& _vel, const bool _stable);
};

} // namespace decision
} // namespace controller
} // namespace ai

#endif // AI_SERVER_CONTROLLER_DETAIL_VELOCITY_GENERATOR_H
