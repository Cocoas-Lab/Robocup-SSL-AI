#ifndef AI_SERVER_UTIL_TIME_HPP_
#define AI_SERVER_UTIL_TIME_HPP_

#include "decision/time.hpp"

namespace ai {
namespace util {

/// プロジェクト内で用いるclock
using ClockType = std::chrono::high_resolution_clock;

/// プロジェクト内で用いる時間を表現する型
using DurationType = typename ClockType::duration;

/// プロジェクト内で用いる時刻を表現する型
using TimePointType = typename ClockType::time_point;

/// @brief        浮動小数点数で表現された時間[s]をstd::chrono::durationに変換する
/// @param time   変換する時間
/// @return       変換された時間
///
/// VisionやRefBoxがこのような形式で時間を送ってくるので, それを変換するために実装.
/// Durationを明示的に指定しなかった場合, microsecオーダでの変換を行う.
template <class Duration = std::chrono::microseconds,
          std::enable_if_t<decision::isDurationValue<Duration>, std::nullptr_t> = nullptr>
inline constexpr Duration toDuration(double time) {
  using rep    = typename Duration::rep;
  using period = typename Duration::period;
  return Duration{static_cast<rep>((time * period::den) / period::num)};
}

} // namespace util
} // namespace ai

#endif // AI_SERVER_UTIL_TIME_H
