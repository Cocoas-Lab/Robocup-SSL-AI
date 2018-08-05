#include <algorithm>
#include <stdexcept>
#include <variant>
#include "driver.hpp"

namespace ai {

driver::driver(boost::asio::io_service& _ioService, util::DurationType _cycle,
               const model::updater::world& _world, model::teamColor _color)
    : timer_(_ioService), cycle_(_cycle), world_(_world), teamColor_(_color) {
  // タイマが開始されたらdriver::main_loop()が呼び出されるように設定
  timer_.async_wait(
      [this](auto&& _error) { mainLoop(std::forward<decltype(_error)>(_error)); });
}

model::teamColor driver::teamColor() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return teamColor_;
}

void driver::teamColor(model::teamColor _color) {
  std::lock_guard<std::mutex> lock(mutex_);
  teamColor_ = _color;
}

boost::signals2::connection driver::onCommandUpdated(
    const UpdatedSignalType::slot_type& _slot) {
  return commandUpdated_.connect(_slot);
}

void driver::velocityLimit(double _limit) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto&& _meta : robotsMetadata_) std::get<1>(_meta.second)->velocityLimit(_limit);
}

void driver::registerRobot(uint32_t _id, ControllerType _controller, SenderType _sender) {
  std::lock_guard<std::mutex> lock(mutex_);
  robotsMetadata_.emplace(
      _id,
      std::forward_as_tuple(model::command{_id}, std::move(_controller), std::move(_sender)));
}

void driver::unregisterRobot(uint32_t _id) {
  std::lock_guard<std::mutex> lock(mutex_);
  robotsMetadata_.erase(_id);
}

bool driver::registered(uint32_t _id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return robotsMetadata_.count(_id);
}

void driver::updateCommand(const model::command& _command) {
  std::lock_guard<std::mutex> lock(mutex_);

  // ロボットが登録されていなかったらエラー
  const auto id = _command.id();
  if (robotsMetadata_.count(id) == 0) {
    throw std::runtime_error(
        boost::str(boost::format("driver: %1% robot id %2% is not registered") %
                   (static_cast<bool>(teamColor_) ? "yellow" : "blue") % id));
  }

  std::get<0>(robotsMetadata_.at(id)) = _command;
}

void driver::mainLoop(const boost::system::error_code& _error) {
  // TODO: エラーが発生したことを上の階層に伝える仕組みを実装する
  if (_error) return;

  // 処理の開始時刻を記録
  const auto startTime = util::ClockType::now();

  std::lock_guard<std::mutex> lock(mutex_);

  // このループでのWorldModelを生成
  const auto world = world_.value();

  // 登録されたロボットの命令をControllerを通してから送信する
  for (auto&& meta : robotsMetadata_) process(world, meta.second);

  // 処理の開始時刻からcycle_経過した後に再度main_loop()が呼び出されるように設定
  timer_.expires_at(startTime + cycle_);
  timer_.async_wait(
      [this](auto&& _error) { mainLoop(std::forward<decltype(_error)>(_error)); });
}

void driver::process(const model::world& _world, MetadataType& _metadata) {
  auto command  = std::get<0>(_metadata);
  const auto id = command.id();
  const auto robots =
      static_cast<bool>(teamColor_) ? _world.robotsYellow() : _world.robotsBlue();

  // ロボットが検出されていないときは何もしない
  if (robots.count(id) == 0) return;

  // commandの指令値をControllerに通す
  auto controller = [id, &c = *std::get<1>(_metadata), &r = robots.at(id)](auto&& _s) {
    return c(r, std::forward<decltype(_s)>(_s));
  };
  command.vel(std::visit(controller, command.setpoint()));

  // Senderで送信
  std::get<2>(_metadata)->sendCommand(command);

  // 登録された関数があればそれを呼び出す
  commandUpdated_(command);
}

} // namespace ai
