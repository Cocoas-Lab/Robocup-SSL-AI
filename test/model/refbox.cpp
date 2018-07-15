#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include "ai/model/refbox.hpp"
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai::model::refbox::stageName)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai::model::refbox::gameCommand)

BOOST_AUTO_TEST_SUITE(refbox)

// refbox getter and initialization check
BOOST_AUTO_TEST_CASE(test001) {
  ai::model::refbox ref{};

  BOOST_TEST(ref.packetTimestamp().time_since_epoch().count() == 0);
  BOOST_TEST(ref.stage() == ai::model::refbox::stageName::NormalFirstHalfPre);
  BOOST_TEST(ref.stageTimeLeft() == 0);
  BOOST_TEST(ref.command() == ai::model::refbox::gameCommand::Halt);
  BOOST_TEST(ref.teamYellow().name() == "yellow");
  BOOST_TEST(ref.teamBlue().name() == "blue");
  BOOST_TEST(std::get<0>(ref.ballPlacementPosition()) == 0.0);
  BOOST_TEST(std::get<1>(ref.ballPlacementPosition()) == 0.0);
}

// refbox setter check
BOOST_AUTO_TEST_CASE(test002) {
  ai::model::refbox ref{};

  constexpr auto dummyTime =
      ai::util::TimePointType{std::chrono::microseconds{1513688793680551}};
  ref.packetTimestamp(dummyTime);
  BOOST_TEST(ref.packetTimestamp().time_since_epoch().count() ==
             dummyTime.time_since_epoch().count());
  ref.stage(ai::model::refbox::stageName::NormalFirstHalf);
  BOOST_TEST(ref.stage() == ai::model::refbox::stageName::NormalFirstHalf);
  ref.stageTimeLeft(2);
  BOOST_TEST(ref.stageTimeLeft() == 2);
  ref.command(ai::model::refbox::gameCommand::Stop);
  BOOST_TEST(ref.command() == ai::model::refbox::gameCommand::Stop);
  ai::model::teamInfo testYellow{"testYellow"};
  ref.teamYellow(testYellow);
  BOOST_TEST(ref.teamYellow().name() == testYellow.name());
  ai::model::teamInfo testBlue{"testBlue"};
  ref.teamBlue(testBlue);
  BOOST_TEST(ref.teamBlue().name() == testBlue.name());

  ref.ballPlacementPosition({1.23, 4.56});
  BOOST_TEST(std::get<0>(ref.ballPlacementPosition()) == 1.23);
  BOOST_TEST(std::get<1>(ref.ballPlacementPosition()) == 4.56);
}

BOOST_AUTO_TEST_SUITE_END()
