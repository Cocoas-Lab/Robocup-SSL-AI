#include "refbox.hpp"

namespace ai {
namespace model {
refbox::refbox() : teamYellow_("yellow"), teamBlue_("blue"), ballPlacementPosition_{} {
  stageTimeLeft_ = 0;
  stage_         = stageName::NormalFirstHalfPre;
  command_       = gameCommand::Halt;
}

util::TimePointType refbox::packetTimestamp() const {
  return packetTimestamp_;
}

uint32_t refbox::stageTimeLeft() const {
  return stageTimeLeft_;
}

refbox::stageName refbox::stage() const {
  return stage_;
}

refbox::gameCommand refbox::command() const {
  return command_;
}

teamInfo refbox::teamYellow() const {
  return teamYellow_;
}

teamInfo refbox::teamBlue() const {
  return teamBlue_;
}

std::tuple<double, double> refbox::ballPlacementPosition() const {
  return ballPlacementPosition_;
}

void refbox::packetTimestamp(util::TimePointType _timePoint) {
  packetTimestamp_ = _timePoint;
}

void refbox::stageTimeLeft(uint32_t _timeLeft) {
  stageTimeLeft_ = _timeLeft;
}

void refbox::stage(stageName _name) {
  stage_ = _name;
}

void refbox::command(gameCommand _command) {
  command_ = _command;
}

void refbox::teamYellow(const teamInfo& _info) {
  teamYellow_ = _info;
}

void refbox::teamBlue(const teamInfo& _info) {
  teamBlue_ = _info;
}

void refbox::ballPlacementPosition(std::tuple<double, double> _position) {
  ballPlacementPosition_ = std::move(_position);
}

} // namespace model
} // namespace ai
