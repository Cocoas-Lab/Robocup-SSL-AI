#ifndef AI_FILTER_OBSERVER_BALL_HPP_
#define AI_FILTER_OBSERVER_BALL_HPP_

#include "ai/filter/base.hpp"
#include "ai/model/world.hpp"
#include <Eigen/Core>
#include <array>
#include <experimental/optional>

namespace ai {
namespace filter {
namespace observer {
class ball : public base<model::ball, timing::OnUpdated> {
private:
  // 係数群
  static constexpr double fricCoef_     = 0.004;        // 床とボールの摩擦係数
  static constexpr double ballWeight_   = 45.93 / 1000; // ボールの重さ[kg]
  static constexpr double ballRadius_   = 42.67 / 2000; // ボールの半径[m]
  static constexpr double airViscosity_ = 1.822e-5;     // 空気の粘度[Pa・s]
  static constexpr double airRegistance_ =
      6 * M_PI * airViscosity_ * ballRadius_; // ボールの粘性抵抗係数[kg/s]
  static constexpr double friction_ =
      fricCoef_ * ballWeight_ * 9.8; // 床とボールの最大動摩擦力[N]
  static constexpr double quantLimitX_ =
      6000 / 1000; // カメラの量子化限界(幅方向)。フィールド幅[mm] / カメラ分解能
  static constexpr double quantLimitY_ =
      9000 / 1000; // カメラの量子化限界(奥行き方向)。フィールド奥行き[mm] / カメラ分解能
  static constexpr double lambdaObserver_ = -9; // ボールの状態オブザーバの極

  model::ball ball_;
  util::TimePointType prevTime_;                    // 前回呼び出された時刻
  std::array<Eigen::Matrix<double, 2, 1>, 2> xHat_; // 状態変数行列 [位置, 速度]T

public:
  ball() = delete;

  /// @brief コンストラクタ
  /// @param ball ボールの初期位置
  /// @param time インスタンス生成時の時刻
  explicit ball(const model::ball& _ball, util::TimePointType _time);
  ball(const ball&) = default;

  /// @brief	状態オブザーバの状態を更新する
  /// @param ball Visionが観測したのボール情報
  /// @param time Visionがフレームをキャプチャした時刻
  /// @return 状態推定後のボール情報
  model::ball update(const model::ball& _ball, util::TimePointType _time) override;

  virtual ~ball() = default;
};
} // namespace observer
} // namespace filter
} // namespace ai

#endif // AI_SERVER_FILTER_STATE_OBSERVER_H
