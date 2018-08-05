#ifndef AI_FILTER_VA_HPP_
#define AI_FILTER_VA_HPP_

#include <chrono>

#include "base.hpp"

namespace ai {
namespace filter {

template <class T>
class va : public base<T, timing::OnUpdated> {
  using ValueType     = T;
  using TimePointType = util::TimePointType;

public:
  va();

  ValueType update(const ValueType&, TimePointType) override;

private:
  TimePointType prevTime_;
  ValueType prevState_;
};

template <class T>
va<T>::va() : prevTime_(TimePointType::min()) {}

} // namespace filter
} // namespace ai

#endif // AI_SERVER_FILTER_VA_CALCULATOR_H
