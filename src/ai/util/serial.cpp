#include "serial.hpp"

namespace ai {
namespace util {
serial::serial(boost::asio::io_service& _ioService, const std::string& _device)
    : _serial(_ioService, _device) {
  _serial.set_option(boost::asio::serial_port::baud_rate(57600));
}

unsigned int serial::baudRate() {
  boost::asio::serial_port::baud_rate baud;
  _serial.get_option(baud);
  return baud.value();
}

unsigned int serial::frameSize() {
  boost::asio::serial_port::character_size size;
  _serial.get_option(size);
  return size.value();
}

serial::FlowControl serial::flowControl() {
  boost::asio::serial_port::flow_control flow;
  _serial.get_option(flow);
  return flow.value();
}

serial::StopBits serial::stopBits() {
  boost::asio::serial_port::stop_bits bit;
  _serial.get_option(bit);
  return bit.value();
}

serial::Parity serial::parity() {
  boost::asio::serial_port::parity parityType;
  _serial.get_option(parityType);
  return parityType.value();
}

void serial::setBaudRate(unsigned int _baud) {
  _serial.set_option(boost::asio::serial_port::baud_rate(_baud));
}

void serial::setFrameSize(unsigned int _size) {
  _serial.set_option(boost::asio::serial_port::character_size(_size));
}

void serial::setFlowControl(serial::FlowControl _flow) {
  _serial.set_option(boost::asio::serial_port::flow_control(_flow));
}

void serial::setParity(serial::Parity _parity) {
  _serial.set_option(boost::asio::serial_port::parity(_parity));
}

void serial::setStopBits(serial::StopBits _bits) {
  _serial.set_option(boost::asio::serial_port::stop_bits(_bits));
}
} // namespace util
} // namespace ai
