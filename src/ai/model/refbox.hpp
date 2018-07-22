#ifndef AI_MODEL_REFBOX_HPP_
#define AI_MODEL_REFBOX_HPP_

#include <tuple>
#include <stdint.h>

#include "ai/util/time.hpp"
#include "teamInfo.hpp"

#include "ssl-protos/refbox/referee.pb.h"

namespace ai {
namespace model {
class refbox {
public:
  enum class stageName {
    NormalFirstHalfPre   = ssl_protos::refbox::Referee_Stage_NORMAL_FIRST_HALF_PRE,
    NormalFirstHalf      = ssl_protos::refbox::Referee_Stage_NORMAL_FIRST_HALF,
    NormalHalfTime       = ssl_protos::refbox::Referee_Stage_NORMAL_HALF_TIME,
    NormalSecondHalfPre  = ssl_protos::refbox::Referee_Stage_NORMAL_SECOND_HALF_PRE,
    NormalSecondHalf     = ssl_protos::refbox::Referee_Stage_NORMAL_SECOND_HALF,
    ExtraTimeBreak       = ssl_protos::refbox::Referee_Stage_EXTRA_TIME_BREAK,
    ExtraFirstHalfPre    = ssl_protos::refbox::Referee_Stage_EXTRA_FIRST_HALF_PRE,
    ExtraFirstHalf       = ssl_protos::refbox::Referee_Stage_EXTRA_FIRST_HALF,
    ExtraHalfTime        = ssl_protos::refbox::Referee_Stage_EXTRA_HALF_TIME,
    ExtraSecondHalfPre   = ssl_protos::refbox::Referee_Stage_EXTRA_SECOND_HALF_PRE,
    ExtraSecondHalf      = ssl_protos::refbox::Referee_Stage_EXTRA_SECOND_HALF,
    PenaltyShootoutBreak = ssl_protos::refbox::Referee_Stage_PENALTY_SHOOTOUT_BREAK,
    PenaltyShootout      = ssl_protos::refbox::Referee_Stage_PENALTY_SHOOTOUT,
    PostGame             = ssl_protos::refbox::Referee_Stage_POST_GAME,
  };

  enum class gameCommand {
    Halt                 = ssl_protos::refbox::Referee_Command_HALT,
    Stop                 = ssl_protos::refbox::Referee_Command_STOP,
    NormalStart          = ssl_protos::refbox::Referee_Command_NORMAL_START,
    ForceStart           = ssl_protos::refbox::Referee_Command_FORCE_START,
    PrepareKickoffYellow = ssl_protos::refbox::Referee_Command_PREPARE_KICKOFF_YELLOW,
    PrepareKickoffBlue   = ssl_protos::refbox::Referee_Command_PREPARE_KICKOFF_BLUE,
    PreparePenaltyYellow = ssl_protos::refbox::Referee_Command_PREPARE_PENALTY_YELLOW,
    PreparePenaltyBlue   = ssl_protos::refbox::Referee_Command_PREPARE_PENALTY_BLUE,
    DirectFreeYellow     = ssl_protos::refbox::Referee_Command_DIRECT_FREE_YELLOW,
    DirectFreeBlue       = ssl_protos::refbox::Referee_Command_DIRECT_FREE_BLUE,
    IndirectFreeYellow   = ssl_protos::refbox::Referee_Command_INDIRECT_FREE_YELLOW,
    IndirectFreeBlue     = ssl_protos::refbox::Referee_Command_INDIRECT_FREE_BLUE,
    TimeoutYellow        = ssl_protos::refbox::Referee_Command_TIMEOUT_YELLOW,
    TimeoutBlue          = ssl_protos::refbox::Referee_Command_TIMEOUT_BLUE,
    GoalYellow           = ssl_protos::refbox::Referee_Command_GOAL_YELLOW,
    GoalBlue             = ssl_protos::refbox::Referee_Command_GOAL_BLUE,
    BallPlacementYellow  = ssl_protos::refbox::Referee_Command_BALL_PLACEMENT_YELLOW,
    BallPlacementBlue    = ssl_protos::refbox::Referee_Command_BALL_PLACEMENT_BLUE,
  };

  refbox();

  util::TimePointType packetTimestamp() const;
  uint32_t stageTimeLeft() const;
  stageName stage() const;
  gameCommand command() const;
  teamInfo teamYellow() const;
  teamInfo teamBlue() const;
  std::tuple<double, double> ballPlacementPosition() const;

  void packetTimestamp(util::TimePointType _timePoint);
  void stageTimeLeft(uint32_t _timeLeft);
  void stage(stageName _name);
  void command(gameCommand _command);
  void teamYellow(const teamInfo& _info);
  void teamBlue(const teamInfo& _info);
  void ballPlacementPosition(std::tuple<double, double> _position);

private:
  util::TimePointType packetTimestamp_;
  uint32_t stageTimeLeft_;
  stageName stage_;
  gameCommand command_;
  teamInfo teamYellow_;
  teamInfo teamBlue_;
  std::tuple<double, double> ballPlacementPosition_;
};
} // namespace model
} // namespace ai

#endif
