#define BOOST_TEST_DYN_LINK

#include <limits>
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai/model/updater/refbox.hpp"
#include "ai/util/math/affine.hpp"
#include "ssl-protos/refbox/referee.pb.h"

BOOST_TEST_DONT_PRINT_LOG_VALUE(ai::model::refbox::gameCommand)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai::model::refbox::stageName)

BOOST_AUTO_TEST_SUITE(updater_refbox)

BOOST_AUTO_TEST_CASE(normal) {
  ai::model::updater::refbox ru;

  {
    // デフォルトコンストラクが呼ばれたときに, 内部の値がちゃんと初期化されているか
    const auto r1 = ru.value();
    const auto r2 = ai::model::refbox{};

    BOOST_TEST(r1.packetTimestamp().time_since_epoch().count() ==
               r2.packetTimestamp().time_since_epoch().count());
    BOOST_TEST(r1.stageTimeLeft() == r2.stageTimeLeft());
    BOOST_TEST(r1.stage() == r2.stage());
    BOOST_TEST(r1.command() == r2.command());

    BOOST_TEST(r1.teamBlue().name() == r2.teamBlue().name());
    BOOST_TEST(r1.teamYellow().name() == r2.teamYellow().name());
  }

  ssl_protos::refbox::Referee referee{};
  {
    referee.set_packet_timestamp(1513688793680551);
    referee.set_stage(ssl_protos::refbox::Referee::Stage::Referee_Stage_NORMAL_FIRST_HALF_PRE);
    // stage_time_leftはoptionalなので送られてこない場合がある
    // referee.set_stage_time_left(2);
    referee.set_command_counter(3);
    referee.set_command(ssl_protos::refbox::Referee::Command::Referee_Command_HALT);
    referee.set_command_timestamp(4);

    ssl_protos::refbox::Referee_TeamInfo blue{};
    blue.set_name("blue");
    blue.set_score(10);
    blue.set_goalie(11);
    blue.set_red_cards(12);
    blue.set_yellow_cards(13);
    // yellow_card_timesは送られてこない可能性があるので,
    // そのテストとして値をセットしないこととする
    // blue.add_yellow_card_times(14);
    blue.set_timeouts(15);
    blue.set_timeout_time(16);
    referee.mutable_blue()->CopyFrom(blue);

    ssl_protos::refbox::Referee_TeamInfo yellow{};
    yellow.set_name("yellow");
    yellow.set_score(21);
    yellow.set_goalie(22);
    yellow.set_red_cards(23);
    yellow.set_yellow_cards(24);
    // yellow_card_timesは複数送られてくる場合がある
    yellow.add_yellow_card_times(25);
    yellow.add_yellow_card_times(26);
    yellow.add_yellow_card_times(27);
    yellow.set_timeouts(28);
    yellow.set_timeout_time(29);
    referee.mutable_yellow()->CopyFrom(yellow);
  }
  BOOST_REQUIRE_NO_THROW(ru.update(referee));

  {
    const auto r = ru.value();

    BOOST_TEST(r.packetTimestamp().time_since_epoch().count() ==
               ai::util::TimePointType{std::chrono::microseconds{1513688793680551}}
                   .time_since_epoch()
                   .count());
    // 送られてこなかったらmax()
    BOOST_TEST(r.stageTimeLeft() == std::numeric_limits<decltype(r.stageTimeLeft())>::max());
    BOOST_TEST(r.stage() == ai::model::refbox::stageName::NormalFirstHalfPre);
    BOOST_TEST(r.command() == ai::model::refbox::gameCommand::Halt);

    const auto b = r.teamBlue();
    BOOST_TEST(b.name() == referee.blue().name());
    BOOST_TEST(b.score() == referee.blue().score());
    BOOST_TEST(b.goalie() == referee.blue().goalie());
    BOOST_TEST(b.redCards() == referee.blue().red_cards());
    BOOST_TEST(b.yellowCards() == referee.blue().yellow_cards());
    // 送られてこなかったら0
    BOOST_TEST(b.yellowCardTimes() == 0);
    BOOST_TEST(b.timeouts() == referee.blue().timeouts());
    BOOST_TEST(b.timeoutTimes() == referee.blue().timeout_time());

    const auto y = r.teamYellow();
    BOOST_TEST(y.name() == referee.yellow().name());
    BOOST_TEST(y.score() == referee.yellow().score());
    BOOST_TEST(y.goalie() == referee.yellow().goalie());
    BOOST_TEST(y.redCards() == referee.yellow().red_cards());
    BOOST_TEST(y.yellowCards() == referee.yellow().yellow_cards());
    // 複数送られてきたら先頭の値
    BOOST_TEST(y.yellowCardTimes() == referee.yellow().yellow_card_times(0));
    BOOST_TEST(y.timeouts() == referee.yellow().timeouts());
    BOOST_TEST(y.timeoutTimes() == referee.yellow().timeout_time());
  }

  // stage_time_leftを設定してみる
  referee.set_stage_time_left(123);
  BOOST_REQUIRE_NO_THROW(ru.update(referee));
  BOOST_TEST(ru.value().stageTimeLeft() == 123);
}

BOOST_AUTO_TEST_CASE(abp) {
  ssl_protos::refbox::Referee referee{};
  {
    referee.set_packet_timestamp(1513688793680551);
    referee.set_stage(ssl_protos::refbox::Referee::Stage::Referee_Stage_NORMAL_FIRST_HALF_PRE);
    referee.set_command_counter(3);
    referee.set_command(
        ssl_protos::refbox::Referee::Command::Referee_Command_BALL_PLACEMENT_BLUE);
    referee.set_command_timestamp(4);

    ssl_protos::refbox::Referee_TeamInfo blue{};
    blue.set_name("blue");
    blue.set_score(10);
    blue.set_goalie(11);
    blue.set_red_cards(12);
    blue.set_yellow_cards(13);
    blue.set_timeouts(15);
    blue.set_timeout_time(16);
    referee.mutable_blue()->CopyFrom(blue);

    ssl_protos::refbox::Referee_TeamInfo yellow{};
    yellow.set_name("yellow");
    yellow.set_score(21);
    yellow.set_goalie(22);
    yellow.set_red_cards(23);
    yellow.set_yellow_cards(24);
    yellow.set_timeouts(28);
    yellow.set_timeout_time(29);
    referee.mutable_yellow()->CopyFrom(yellow);

    ssl_protos::refbox::Referee_Point point{};
    point.set_x(100);
    point.set_y(200);
    referee.mutable_designated_position()->CopyFrom(point);
  }

  {
    // 座標変換なし
    ai::model::updater::refbox ru{};
    ru.update(referee);

    const auto& r = ru.value();
    BOOST_TEST(std::get<0>(r.ballPlacementPosition()) = 100.0);
    BOOST_TEST(std::get<1>(r.ballPlacementPosition()) = 200.0);
  }

  {
    // 座標変換あり
    using namespace boost::math::double_constants;
    const auto mat = ai::util::math::makeTransformationMatrix(10.0, 20.0, half_pi);

    ai::model::updater::refbox ru{};
    ru.transformationMatrix(mat);
    ru.update(referee);

    const auto& r = ru.value();
    BOOST_TEST(std::get<0>(r.ballPlacementPosition()) = 200.0 + 10);
    BOOST_TEST(std::get<1>(r.ballPlacementPosition()) = 100.0 + 20);
  }
}

BOOST_AUTO_TEST_SUITE_END()
