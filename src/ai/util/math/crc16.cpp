#include "crc16.hpp"
#include <iostream>

namespace ai{
	namespace util{
		namespace math{
			uint16_t crc16(const std::vector<uint8_t>& _buf){
				uint16_t result = 0xFFFF;
				constexpr uint16_t poly = 0x8005;

				auto frameCrc = [&result](const uint8_t _frame)->auto{
					result ^= (_frame << 8);
					for(int i = 0;i < 8;i++){
						if(result & 0x8000){
							result <<= 1;
							result ^= poly;
						}else{
							result <<= 1;
						}
					}
				};

				for(auto buf: _buf){
					frameCrc(buf);
				}

				return result;
			}
		}
	}
}
