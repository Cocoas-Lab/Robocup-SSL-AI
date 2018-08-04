#ifndef AI_DRIVER_HPP_
#define AI_DRIVER_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <tuple>
#include <stdint.h>
#include <boost/asio.hpp>
#include <boost/asio/basic_waitable_timer.hpp>
#include <boost/signals2.hpp>

#include "ai/controller/base.hpp"
#include "ai/model/command.hpp"
#include "ai/model/teamColor.hpp"
#include "ai/model/updater/world.hpp"
#include "ai/sender/base.hpp"
#include "ai/util/time.hpp"

namespace ai {
class driver {
  /// Controllerのポインタの型
  using ControllerType = std::unique_ptr<controller::base>;
  /// Senderのポインタの型
  using SenderType = std::shared_ptr<sender::base>;
  /// Driverで行う処理で必要となる各ロボットの情報の型
  using MetadataType = std::tuple<model::command, ControllerType, SenderType>;
  /// Commandが更新された(Controllerを通された)ときに発火するシグナルの型
  using UpdatedSignalType = boost::signals2::signal<void(const model::command&)>;

public:
  /// @param _cycle            制御周期
  /// @param _world            updater::worldの参照
  /// @param _color            チームカラー
  driver(boost::asio::io_service& _ioService, util::DurationType _cycle,
         const model::updater::world& _world, model::teamColor _color);

  /// @brief                  現在設定されているチームカラーを取得する
  model::teamColor teamColor() const;

  /// @brief                  チームカラーを変更する
  /// @param _color            チームカラー
  void teamColor(model::teamColor _color);

  /// @brief                  Driverにロボットを登録する
  /// @param _id               ロボットのID
  /// @param _controller       Controller
  /// @param _sender           Sender
  void registerRobot(uint32_t _id, ControllerType _controller, SenderType _sender);

  /// @brief                  Driverに登録されたロボットを解除する
  /// @param _id               ロボットのID
  void unregisterRobot(uint32_t _id);

  /// @brief                  ロボットがDriverに登録されているか調べる
  /// @param _id               ロボットのID
  bool registered(uint32_t _id) const;

  /// @brief                  ロボットへの命令を更新する
  /// @param _command          ロボットへの命令
  void updateCommand(const model::command& _command);

  /// @brief                  commandが更新されたときに呼ばれる関数を登録する
  /// @param _slot             commandが更新されたときに呼びたい関数
  boost::signals2::connection onCommandUpdated(const UpdatedSignalType::slot_type& _slot);

  /// @brief                  登録されているロボットのControllerに速度制限をかける
  /// @param _limit            速度の制限値
  void velocityLimit(double _limit);

private:
  /// @brief                  cycle_毎に呼ばれる制御部のメインループ
  void mainLoop(const boost::system::error_code& _error);

  /// @brief                  ロボットへの命令をControllerを通してから送信する
  void process(const model::world& _world, MetadataType& _metadata);

  mutable std::mutex mutex_;

  /// 制御部の処理を一定の周期で回すためのタイマ
  boost::asio::basic_waitable_timer<util::ClockType> timer_;
  /// 制御周期
  util::DurationType cycle_;

  /// updater::worldの参照
  const model::updater::world& world_;

  /// チームカラー
  model::teamColor teamColor_;

  /// 登録されたロボットの情報
  std::unordered_map<uint32_t, MetadataType> robotsMetadata_;

  UpdatedSignalType commandUpdated_;
};
} // namespace ai

#endif
