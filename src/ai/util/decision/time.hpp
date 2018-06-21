#ifndef AI_SERVER_UTIL_DECISION_TIME_HPP_
#define AI_SERVER_UTIL_DECISION_TIME_HPP_

#include <chrono>
#include <type_traits>

namespace ai {
namespace util {
namespace decision {

// template型引数に与えた型がstd::chrono::durationかを判定するメタ関数
template <class>
struct isDuration : std::false_type {};

template <class Rep, class Period>
struct isDuration<std::chrono::duration<Rep, Period>> : std::true_type {};

template <class T>
static constexpr auto isDurationValue = isDuration<T>::value;

} // namespace decision
} // namespace util
} // namespace ai

#endif // AI_SERVER_UTIL_DETAIL_TIME_H
