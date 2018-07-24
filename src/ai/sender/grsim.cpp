#include <variant>
#include <ostream>
#include <tuple>

#include "grsim.hpp"

namespace ai {
namespace sender {

grsim::grsim(boost::asio::io_service& _ioService, const std::string& _grsimAddr, uint16_t _port)
    : udpSender_(_ioService, _grsimAddr, _port) {
  //プロトコルバッファのバージョン確認
  GOOGLE_PROTOBUF_VERIFY_VERSION;
}

void grsim::sendCommand(const model::command& _command) {
  ssl_protos::grsim::Packet packet{};

  //パケットに値をセット
  auto commands = packet.mutable_commands();
  commands->set_isteamyellow(true);
  commands->set_timestamp(0.0);

  auto grcommand = commands->add_robot_commands();

  grcommand->set_id(_command.id());

  const auto& kickFlag  = std::get<0>(_command.kick());
  const auto& kickPower = std::get<1>(_command.kick());

  using kickType = model::command::kickType;

  if (kickFlag == kickType::Straight) {
    grcommand->set_kickspeedx(kickPower);
    grcommand->set_kickspeedz(0);
  } else if (kickFlag == kickType::Tip) {
    grcommand->set_kickspeedx(0);
    grcommand->set_kickspeedz(kickPower);
  } else {
    grcommand->set_kickspeedx(0);
    grcommand->set_kickspeedz(0);
  }

  const auto& setpoint = _command.setpoint();
  if (const auto& velocity = std::get_if<model::command::velocity>(&setpoint)) {
    // velocity_tへのキャストが成功した時
    grcommand->set_veltangent(velocity->vx / 1000);
    grcommand->set_velnormal(velocity->vy / 1000);
    grcommand->set_velangular(velocity->omega);
  } else {
    // velocity_tへのキャストが失敗した時
    grcommand->set_veltangent(0);
    grcommand->set_velnormal(0);
    grcommand->set_velangular(0);
  }

  grcommand->set_spinner(_command.dribble() != 0);
  grcommand->set_wheelsspeed(false);

  //送信バッファと、それに書き込むstreamクラスのオブジェクトを作成
  boost::asio::streambuf buf;
  std::ostream os(&buf);

  // packetをシリアライズし、結果をos経由でbufに書き込む
  packet.SerializeToOstream(&os);

  // bufを送信
  udpSender_.send(buf.data());
}
} // namespace sender
} // namespace ai
