#ifndef AI_MODEL_FIELD_HPP_
#define AI_MODEL_FIELD_HPP_

#include <stdint.h>

namespace ai {
namespace model {
class field {
private:
  uint32_t length_;
  uint32_t width_;
  uint32_t centerRadius_;
  uint32_t goalWidth_;
  uint32_t penaltyLength_;
  uint32_t penaltyWidth_;

public:
  field();

  uint32_t length() const;
  uint32_t width() const;
  uint32_t centerRadius() const;
  uint32_t goalWidth() const;
  uint32_t penaltyLength() const;
  uint32_t penaltyWidth() const;

  void length(uint32_t _length);
  void width(uint32_t _width);
  void centerRadius(uint32_t _centerRadius);
  void goalWidth(uint32_t _goalWidth);
  void penaltyLength(uint32_t _penaltyLength);
  void penaltyWidth(uint32_t _penaltyWidth);

  double xMax() const;
  double xMin() const;
  double yMax() const;
  double yMin() const;
};
} // namespace model
} // namespace ai

#endif
