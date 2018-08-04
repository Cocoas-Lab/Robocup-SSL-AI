#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include "ai/model/command.hpp"

BOOST_TEST_DONT_PRINT_LOG_VALUE(ai::model::command::kickType)

BOOST_AUTO_TEST_SUITE(command)

// command getter and initialization check
BOOST_AUTO_TEST_CASE(test001) {
  ai::model::command command{0};

  BOOST_TEST(command.id() == 0);
  BOOST_TEST(command.dribble() == 0);
  auto kickFlagTest = command.kick();
  BOOST_TEST(std::get<0>(kickFlagTest) == ai::model::command::kickType::None);
  BOOST_TEST(std::get<1>(kickFlagTest) == 0.0);
  const auto& testVelocity = std::get<ai::model::command::velocity>(command.setpoint());
  BOOST_TEST(testVelocity.vx == 0.0);
  BOOST_TEST(testVelocity.vy == 0.0);
  BOOST_TEST(testVelocity.omega == 0.0);
}

// command setter check
BOOST_AUTO_TEST_CASE(test002) {
  ai::model::command command{0};

  command.dribble(1);
  BOOST_TEST(command.dribble() == 1);
  ai::model::command::KickFlag kickFlagTest(ai::model::command::kickType::Straight, 2.0);
  command.kick(kickFlagTest);
  auto kickFlagTest2 = command.kick();
  BOOST_TEST(std::get<0>(kickFlagTest2) == ai::model::command::kickType::Straight);
  BOOST_TEST(std::get<1>(kickFlagTest2) == 2.0);
  ai::model::command::position position{3.0, 4.0, 5.0};
  command.pos(position);
  const auto& testPosition = std::get<ai::model::command::position>(command.setpoint());
  BOOST_TEST(testPosition.x == 3.0);
  BOOST_TEST(testPosition.y == 4.0);
  BOOST_TEST(testPosition.theta == 5.0);
  ai::model::command::velocity velocity{6.0, 7.0, 8.0};
  command.vel(velocity);
  const auto& testVelocity = std::get<ai::model::command::velocity>(command.setpoint());
  BOOST_TEST(testVelocity.vx == 6.0);
  BOOST_TEST(testVelocity.vy == 7.0);
  BOOST_TEST(testVelocity.omega == 8.0);
}

BOOST_AUTO_TEST_SUITE_END()
