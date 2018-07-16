#include <functional>
#include "refbox.hpp"

namespace ai {
namespace receiver {

refbox::refbox(boost::asio::io_service& _ioService, const std::string& _listenAddr,
               const std::string& _multicastAddr, uint16_t _port)
    : receiver_(_ioService, _listenAddr, _multicastAddr, _port) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  receiver_.onReceive(
      std::bind(&refbox::parsePacket, this, std::placeholders::_1, std::placeholders::_2));
}

boost::signals2::connection refbox::onReceive(const ReceiveSignalType::slot_type& _slot) {
  return received_.connect(_slot);
}

boost::signals2::connection refbox::onError(const ErrorSignalType::slot_type& _slot) {
  return errored_.connect(_slot);
}

void refbox::parsePacket(const util::multicast::receiver::Buffer& _buffer, std::size_t _size) {
  ssl_protos::refbox::Referee packet;

  if (packet.ParseFromArray(_buffer.data(), _size)) {
    received_(packet);
  } else {
    errored_();
  }
}
} // namespace receiver
} // namespace ai
