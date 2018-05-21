#include <functional>
#include "receiver.hpp"

#include <iostream>

namespace ai{
	namespace util{
		namespace multicast{
			receiver::receiver(boost::asio::io_service& _ioService, const std::string& _listen, const std::string& _multicast, uint16_t _port):
					receiver(_ioService, boost::asio::ip::address::from_string(_listen), boost::asio::ip::address::from_string(_multicast), _port){
			}

			receiver::receiver(boost::asio::io_service& _ioService, const boost::asio::ip::address& _listen, const boost::asio::ip::address& _multicast, uint16_t _port):
					socket(_ioService), endPoint(_listen, _port){
				socket.open(endPoint.protocol());
				socket.set_option(UDP::socket::reuse_address(true));
				socket.bind(endPoint);
				socket.set_option(boost::asio::ip::multicast::join_group(_multicast));
				boost::asio::spawn(socket.get_io_service(), std::bind(&receiver::receive, this, std::placeholders::_1));
			}

			boost::signals2::connection receiver::onReceive(const ReceiveSignal::slot_type& _slot){
				return received.connect(_slot);
			}

			boost::signals2::connection receiver::onError(const ErrorSignal::slot_type& _slot){
				return errored.connect(_slot);
			}

			void receiver::receive(boost::asio::yield_context _yield){
				boost::system::error_code errorCode;

				while(true){
					Buffer buf;
					const auto rxSize = socket.async_receive_from(boost::asio::buffer(buf), endPoint, _yield[errorCode]);

					if(errorCode){
						errored(errorCode);
					}else{
						received(buf, rxSize);
					}
				}
			}
		}
	}
}
