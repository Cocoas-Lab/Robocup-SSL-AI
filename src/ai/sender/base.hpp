#ifndef AI_SENDER_BASE_HPP_
#define AI_SENDER_BASE_HPP_

#include "ai/model/command.hpp"

namespace ai {
namespace sender {

class base {
public:
  virtual ~base() = default;

  virtual void sendCommand(const model::command& _command) = 0;
};

} // namespace sender
} // namespace ai

#endif // AI_SERVER_SENDER_BASE_H
