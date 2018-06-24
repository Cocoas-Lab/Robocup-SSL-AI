#include "teamInfo.hpp"

namespace ai{
	namespace model{
		teamInfo::teamInfo(const std::string& _name):name_(_name), score_(0), goalie_(0), redCards_(0), yellowCards_(0), yellowCardTimes_(0), timeouts_(0), timeoutTimes_(0){}

const std::string& teamInfo::name() const {
  return name_;
}

uint32_t teamInfo::score() const {
  return score_;
}

uint32_t teamInfo::goalie() const {
  return goalie_;
}

uint32_t teamInfo::redCards() const {
  return redCards_;
}

uint32_t teamInfo::yellowCards() const {
  return yellowCards_;
}

uint32_t teamInfo::yellowCardTimes() const {
  return yellowCardTimes_;
}

uint32_t teamInfo::timeouts() const {
  return timeouts_;
}

uint32_t teamInfo::timeoutTimes() const {
  return timeoutTimes_;
}

void teamInfo::score(uint32_t _score) {
  score_ = _score;
}

void teamInfo::goalie(uint32_t _goalie) {
  goalie_ = _goalie;
}

void teamInfo::redCards(uint32_t _redCards) {
  redCards_ = _redCards;
}

void teamInfo::yellowCards(uint32_t _yellowCards) {
  yellowCards_ = _yellowCards;
}

void teamInfo::yellowCardTimes(uint32_t _yellowCardTimes) {
  yellowCardTimes_ = _yellowCardTimes;
}

void teamInfo::timeouts(uint32_t _timeouts) {
  timeouts_ = _timeouts;
}

void teamInfo::timeoutTimes(uint32_t _timeoutTimes) {
  timeoutTimes_ = _timeoutTimes;
}
	}
}
