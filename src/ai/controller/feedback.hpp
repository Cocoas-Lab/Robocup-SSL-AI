#ifndef AI_CONTROLLER_FEEDBACK_HPP_
#define AI_CONTROLLER_FEEDBACK_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "ai/controller/decision/predictor.hpp"
#include "ai/controller/decision/velGen.hpp"
#include "ai/model/world.hpp"
#include "base.hpp"

namespace ai {
namespace controller {

class feedback : public base {
  /* ------------------------------------
  ロボットの状態を3*3行列で表す
         x,    vx,    ax
         y,    vy,    ay
     theta, omega, alpha
  各要素を列ベクトルで
  ------------------------------------ */

private:
  double cycle_;                   // 制御周期
  const model::world& world_;      // worldmodel
  const static double k_;          // 極,収束の速さ
  static const double zeta_;       // モデルパラメータζ
  static const double omega_;      // モデルパラメータω
  static const double vMax_;       // 最大速度
  Eigen::Vector3d kp_;             // 比例ゲイン(x,y,rotate)
  Eigen::Vector3d ki_;             // 積分ゲイン(x,y,rotate)
  Eigen::Vector3d kd_;             // 微分ゲイン(x,y,rotate)
  Eigen::Matrix3d estimatedRobot_; // 推定ロボット状態
  Eigen::Vector3d up_[2];          // 操作量(比例,1フレーム前まで)
  Eigen::Vector3d ui_[2];          // 操作量(積分,1フレーム前まで)
  Eigen::Vector3d ud_[2];          // 操作量(微分,1フレーム前まで)
  Eigen::Vector3d u_[2];           // 操作量(1フレーム前まで)
  Eigen::Vector3d e_[2];           // 偏差(1フレーム前まで)
  decision::velGen generator_;
  decision::predictor predictor_;

  // レギュレータ部
  void calcRegulator(const model::robot& _robot);

  // フィールド基準座標系からロボット基準座標系に変換
  Eigen::Vector3d convert(const Eigen::Vector3d& _raw, const double _robotTheta);

  // 出力計算及び後処理
  void calcOutput(Eigen::Vector3d _target, double _targetAngle);

  // 2点と角度から,2点を通る直線と原点を通る指定角度の直線との交点を求め
  // 原点から交点までの距離を返す
  double findCrossPoint(const double _x1, const double _y2, const double _angle);

public:
  // コンストラクタ
  explicit feedback(double _cycle, const model::world& _world);

  void velocityLimit(const double _limit) override;

  // 制御入力更新関数
  velocity update(const model::robot& _robot, const position& _setpoint) override;
  velocity update(const model::robot& _robot, const velocity& _setpoint) override;
};

} // namespace controller
} // namespace ai

#endif // AI_SERVER_CONTROLLER_STATE_FEEDBACK_CONTROLLER_H
