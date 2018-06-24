#include "crc16.hpp"
#include <iostream>

namespace ai {
namespace util {
namespace math {
uint16_t crc16(const std::vector<uint8_t>& _buf) {
  uint16_t result         = 0xFFFF;
  constexpr uint16_t poly = 0x8005; // CRC生成多項式

  // 1frameのCRCを計算するためのlambda
  auto frameCrc = [&result](const uint8_t _frame) -> auto {
    result ^= (_frame << 8);

    for (int i = 0; i < 8; i++) {
      if (result & 0x8000) {
        // シフト時にキャリーが発生する場合は多項式でXOR
        result <<= 1;
        result ^= poly;
      } else {
        result <<= 1;
      }
    }
  };

  // 全てのframeに対してCRCを計算
  for (auto buf : _buf) {
    frameCrc(buf);
  }

  return result;
}
} // namespace math
} // namespace util
} // namespace ai
