#include "sender.hpp"

namespace ai{
	namespace util{
		namespace multicast{
			sender::sender(boost::asio::io_service& _ioService, const std::string& _multicast, uint16_t _port):
						sender(_ioService, boost::asio::ip::address::from_string(_multicast), _port) {}

			sender::sender(boost::asio::io_service& _ioService, const boost::asio::ip::address& _multicast, uint16_t _port):
					udpSocket(_ioService),
					endPoint(_multicast, _port) {
				udpSocket.open(endPoint.protocol());
			}
		}
	}
}
