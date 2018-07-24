#ifndef AI_SENDER_GRSIM_HPP_
#define AI_SENDER_GRSIM_HPP_

#include <string>
#include <boost/asio.hpp>
#include <stdint.h>

#include "ai/model/command.hpp"
#include "ai/util/multicast/sender.hpp"
#include "ssl-protos/grSim/packet.pb.h"

#include "base.hpp"

namespace ai {
namespace sender {

class grsim final : public base {
public:
  grsim(boost::asio::io_service& _ioService, const std::string& _grsimAddr, uint16_t _port);

  void sendCommand(const model::command& _command) override;

private:
  util::multicast::sender udpSender_;
};
} // namespace sender
} // namespace ai
#endif
