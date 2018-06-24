#ifndef AI_UTIL_MULTICAST_SENDER_HPP
#define AI_UTIL_MULTICAST_SENDER_HPP

#include <array>
#include <string>
#include <stdint.h>
#include <boost/asio.hpp>

namespace ai {
namespace util {
namespace multicast {
class sender {
public:
  sender(boost::asio::io_service& _ioService, const std::string& _multicast, uint16_t _port);

  sender(boost::asio::io_service& _ioService, const boost::asio::ip::address& _multicast,
         uint16_t _port);

  template <class Buffer>
  uint32_t send(Buffer&& _buffer) {
    return udpSocket.send_to(boost::asio::buffer(std::forward<Buffer>(_buffer)), endPoint);
  }

  template <class Buffer>
  uint32_t send(Buffer&& _buffer, std::size_t _size) {
    return udpSocket.send_to(boost::asio::buffer(std::forward<Buffer>(_buffer), _size),
                             endPoint);
  }

private:
  boost::asio::ip::udp::socket udpSocket;
  boost::asio::ip::udp::endpoint endPoint;
};
} // namespace multicast
} // namespace util
} // namespace ai

#endif
