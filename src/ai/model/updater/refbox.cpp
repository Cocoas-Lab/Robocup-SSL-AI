#include <limits>
#include <tuple>

#include "ai/util/time.hpp"
#include "ai/util/math/affine.hpp"
#include "refbox.hpp"
#include "ssl-protos/refbox/referee.pb.h"

namespace ai {
namespace model {
namespace updater {

refbox::refbox() : refbox_{}, affine_{Eigen::Translation3d{.0, .0, .0}} {}

void refbox::update(const ssl_protos::refbox::Referee& _referee) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  refbox_.packetTimestamp(
      util::TimePointType{std::chrono::microseconds{_referee.packet_timestamp()}});
  refbox_.stageTimeLeft(_referee.has_stage_time_left()
                            ? _referee.stage_time_left()
                            : std::numeric_limits<decltype(refbox_.stageTimeLeft())>::max());
  refbox_.stage(static_cast<model::refbox::stageName>(_referee.stage()));
  refbox_.command(static_cast<model::refbox::gameCommand>(_referee.command()));

  auto toTeamInfo = [](auto&& _teamInfo) {
    model::teamInfo result{_teamInfo.name()};
    result.score(_teamInfo.score());
    result.goalie(_teamInfo.goalie());
    result.redCards(_teamInfo.red_cards());
    result.yellowCards(_teamInfo.yellow_cards());
    result.yellowCardTimes(
        _teamInfo.yellow_card_times_size() > 0 ? _teamInfo.yellow_card_times(0) : 0);
    result.timeouts(_teamInfo.timeouts());
    result.timeoutTimes(_teamInfo.timeout_time());
    return result;
  };
  refbox_.teamBlue(toTeamInfo(_referee.blue()));
  refbox_.teamYellow(toTeamInfo(_referee.yellow()));

  if (_referee.has_designated_position()) {
    const auto& dp = _referee.designated_position();
    const auto p   = std::make_tuple(static_cast<double>(dp.x()), static_cast<double>(dp.y()));
    refbox_.ballPlacementPosition(util::math::transform(affine_, p));
  }
}

void refbox::transformationMatrix(const Eigen::Affine3d& _matrix) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  affine_ = _matrix;
}

model::refbox refbox::value() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return refbox_;
}

} // namespace updater
} // namespace model
} // namespace ai
