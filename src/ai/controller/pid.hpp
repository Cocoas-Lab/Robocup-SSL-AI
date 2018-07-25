#ifndef AI_CONTROLLER_PID_HPP_
#define AI_CONTROLLER_PID_HPP_

#include "base.hpp"

namespace ai {
namespace controller {

class pid : public base {
private:
  double cycle_;                                         // 制御周期
  model::robot robot_;                                   // ロボット
  static constexpr double kp_[2]           = {2.0, 4.0}; // 比例ゲイン(xy,rotate)
  static constexpr double ki_[2]           = {0.2, 0.2}; // 積分ゲイン(xy,rotate)
  static constexpr double kd_[2]           = {0.0, 0.0}; // 微分ゲイン(xy,rotate)
  static constexpr double maxVelocity_     = 5000.0;     // 最大速度
  static constexpr double maxAcceleration_ = 2000.0;     // 最大加速度
  static constexpr double minAcceleration_ = 100.0;      // 最小加速度
  static constexpr double reachSpeed_      = 1500.0;     // 加速度上昇の速さ
  velocity up_[2]; // 操作量(比例,1フレーム前まで)
  velocity ui_[2]; // 操作量(積分,1フレーム前まで)
  velocity ud_[2]; // 操作量(微分,1フレーム前まで)
  velocity u_[2];  // 操作量(1フレーム前まで)
  velocity e_[2];  // 偏差(1フレーム前まで)
  // 入力制限
  void limitation();

public:
  // コンストラクタ
  explicit pid(double _cycle);

  void velocityLimit(const double _limit) override;
  // 制御入力更新関数
  velocity update(const model::robot& _robot, const position& _setpoint) override;
  velocity update(const model::robot& _robot, const velocity& _setpoint) override;
};

} // namespace controller
} // namespace ai

#endif // AI_SERVER_CONTROLLER_PID_CONTROLLER_H
