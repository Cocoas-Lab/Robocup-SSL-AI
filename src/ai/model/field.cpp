#include "field.hpp"

namespace ai {
namespace model {
field::field()
    : length_(12000),
      width_(9000),
      centerRadius_(500),
      goalWidth_(1200),
      penaltyLength_(1200),
      penaltyWidth_(2400) {}

uint32_t field::length() const {
  return length_;
}

uint32_t field::width() const {
  return width_;
}

uint32_t field::centerRadius() const {
  return centerRadius_;
}

uint32_t field::goalWidth() const {
  return goalWidth_;
}

uint32_t field::penaltyLength() const {
  return penaltyLength_;
}

uint32_t field::penaltyWidth() const {
  return penaltyWidth_;
}

void field::length(uint32_t _length) {
  length_ = _length;
}

void field::width(uint32_t _width) {
  width_ = _width;
}

void field::centerRadius(uint32_t _centerRadius) {
  centerRadius_ = _centerRadius;
}

void field::goalWidth(uint32_t _goalWidth) {
  goalWidth_ = _goalWidth;
}

void field::penaltyLength(uint32_t _penaltyLength) {
  penaltyLength_ = _penaltyLength;
}

void field::penaltyWidth(uint32_t _penaltyWidth) {
  penaltyWidth_ = _penaltyWidth;
}

double field::xMax() const {
  return (length_ / 2);
}

double field::xMin() const {
  return -(length_ / 2);
}

double field::yMax() const {
  return (width_ / 2);
}

double field::yMin() const {
  return -(width_ / 2);
}
} // namespace model
} // namespace ai
