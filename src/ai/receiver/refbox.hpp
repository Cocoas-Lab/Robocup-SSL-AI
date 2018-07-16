#ifndef AI_RECEIVER_REFBOX_HPP_
#define AI_RECEIVER_REFBOX_HPP_

#include <boost/asio.hpp>
#include <boost/signals2.hpp>
#include <stdint.h>

#include "ai/model/refbox.hpp"
#include "ai/util/multicast/receiver.hpp"

#include "ssl-protos/refbox/referee.pb.h"

namespace ai {
namespace receiver {

class refbox {
  // データ受信時に呼ぶシグナルの型
  using ReceiveSignalType = boost::signals2::signal<void(const ssl_protos::refbox::Referee&)>;
  // エラー時に呼ぶシグナルの型
  using ErrorSignalType = boost::signals2::signal<void(void)>;

public:
  refbox(boost::asio::io_service& _ioService, const std::string& _listenAddr,
         const std::string& _multicastAddr, uint16_t _port);
  boost::signals2::connection onReceive(const ReceiveSignalType::slot_type& _slot);
  boost::signals2::connection onError(const ErrorSignalType::slot_type& _slot);

private:
  void parsePacket(const util::multicast::receiver::Buffer& _buffer, std::size_t _size);
  util::multicast::receiver receiver_;

  ReceiveSignalType received_;
  ErrorSignalType errored_;
};
} // namespace receiver
} // namespace ai

#endif
