#ifndef AI_UTIL_MATH_CRC16_HPP
#define AI_UTIL_MATH_CRC16_HPP

#include <functional>
#include <vector>
#include <stdint.h>

namespace ai{
	namespace util{
		namespace math{
			uint16_t crc16(const std::vector<uint8_t>& _buf);
		}
	}
}

#endif
