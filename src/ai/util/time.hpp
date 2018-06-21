#ifndef AI_SERVER_UTIL_TIME_HPP_
#define AI_SERVER_UTIL_TIME_HPP_

#include "detail/time.hpp"

namespace ai {
namespace util {

/// プロジェクト内で用いるclock
using ClockType = std::chrono::high_resolution_clock;

/// プロジェクト内で用いる時間を表現する型
using DurationType = typename clock_type::duration;

/// プロジェクト内で用いる時刻を表現する型
using TimePointType = typename clock_type::time_point;
} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_TIME_H
