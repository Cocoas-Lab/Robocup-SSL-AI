#ifndef AI_UTIL_SERIAL_HPP_
#define AI_UTIL_SERIAL_HPP_

#include <array>
#include <string>
#include <boost/asio.hpp>

namespace ai {
namespace util {
class serial {
public:
  using StopBits    = boost::asio::serial_port::stop_bits::type;
  using Parity      = boost::asio::serial_port::parity::type;
  using FlowControl = boost::asio::serial_port::flow_control::type;

  serial() = default;

  serial(const serial&) = default;

  serial(boost::asio::io_service& _ioService, const std::string& _device);

  unsigned int baudRate();

  unsigned int frameSize();

  FlowControl flowControl();

  StopBits stopBits();

  Parity parity();

  void setBaudRate(const unsigned int _baud);

  void setFrameSize(const unsigned int _size);

  void setFlowControl(const FlowControl _flow);

  void setParity(const Parity _parity);

  void setStopBits(const StopBits _bits);

  template <class Buffer>
  unsigned int write(const Buffer&& _buffer) {
    auto written = 0;
    _serial.write_some(boost::asio::buffer(std::forward<Buffer>(_buffer)));

    // 実際に送信したバイト数を返す
    return written;
  }

  template <class Buffer>
  unsigned int write(const Buffer&& _buffer, const std::size_t _size) {
    return write(boost::asio::buffer(std::forward<Buffer>(_buffer), _size));
  }

private:
  boost::asio::serial_port _serial;
};
} // namespace util
} // namespace ai

#endif
