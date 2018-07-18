#include <functional>
#include "vision.hpp"

namespace ai {
namespace receiver {

vision::vision(boost::asio::io_service& _ioService, const std::string& _listenAddr,
               const std::string& _multicastAddr, uint16_t _port)
    : receiver_(_ioService, _listenAddr, _multicastAddr, _port) {
  // Google Protocol Buffersライブラリのバージョンをチェックする
  // 互換性のないバージョンが使われていた場合は例外吐いて落ちる()
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  // multicastクライアントとparse_packet()をつなげる
  receiver_.onReceive(
      std::bind(&vision::parsePacket, this, std::placeholders::_1, std::placeholders::_2));
}

boost::signals2::connection vision::onReceive(const ReceiveSignalType::slot_type& _slot) {
  return received_.connect(_slot);
}

boost::signals2::connection vision::onError(const ErrorSignalType::slot_type& _slot) {
  return errored_.connect(_slot);
}

void vision::parsePacket(const util::multicast::receiver::Buffer& _buffer, std::size_t _size) {
  ssl_protos::vision::WrapperPacket packet;

  // パケットをパース
  if (packet.ParseFromArray(_buffer.data(), _size)) {
    // 成功したら登録された関数を呼び出す
    received_(packet);
  } else {
    errored_();
  }
}

} // namespace receiver
} // namespace ai
