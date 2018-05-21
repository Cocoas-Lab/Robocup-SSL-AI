#ifndef AI_UTIL_MULTICAST_RECEIVER_HPP
#define AI_UTIL_MULTICAST_RECEIVER_HPP

#include <array>
#include <string>
#include <stdint.h>
#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/signals2.hpp>

namespace ai{
	namespace util{
		namespace multicast{
			class receiver{
				static constexpr std::size_t bufferSize = 8192;

				using Buffer = std::array<uint8_t, bufferSize>;
				using ReceiveSignal = boost::signals2::signal<void(const Buffer&, std::size_t)>;
				using ErrorSignal = boost::signals2::signal<void(const boost::system::error_code&)>;
				using UDP = boost::asio::ip::udp;

				UDP::socket socket;
				UDP::endpoint endPoint;
				ReceiveSignal received;
				ErrorSignal errored;

			public:
				receiver(boost::asio::io_service& _ioService, const std::string& _listen, const std::string& _multicast, uint16_t _port);
				receiver(boost::asio::io_service& _ioService, const boost::asio::ip::address& _listen, const boost::asio::ip::address& _multicast, uint16_t _port);
				boost::signals2::connection onReceive(const ReceiveSignal::slot_type& _slot);
				boost::signals2::connection onError(const ErrorSignal::slot_type& _slot);

			private:
				void receive(boost::asio::yield_context _yield);
			};
		}
	}
}

#endif
