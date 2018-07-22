#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include "ai/model/robot.hpp"

BOOST_AUTO_TEST_SUITE(robot)

BOOST_AUTO_TEST_CASE(test001) {
  // constructor test
  ai::model::robot robot(1, 2.0, 3.0, 4.0);

  // get value test
  BOOST_TEST(robot.id() == 1);
  BOOST_TEST(robot.x() == 2.0);
  BOOST_TEST(robot.y() == 3.0);
  BOOST_TEST(robot.theta() == 4.0);

  // set value test
  robot.x(10.0);
  BOOST_TEST(robot.x() == 10.0);

  robot.y(11.0);
  BOOST_TEST(robot.y() == 11.0);

  robot.vx(12.0);
  BOOST_TEST(robot.vx() == 12.0);

  robot.vy(13.0);
  BOOST_TEST(robot.vy() == 13.0);

  robot.theta(14.0);
  BOOST_TEST(robot.theta() == 14.0);

  robot.omega(15.0);
  BOOST_TEST(robot.omega() == 15.0);

  robot.ax(16.0);
  BOOST_TEST(robot.ax() == 16.0);

  robot.ay(17.0);
  BOOST_TEST(robot.ay() == 17.0);
}

BOOST_AUTO_TEST_CASE(test002) {
  // constructor test
  ai::model::robot robot2(10);

  // get value test
  BOOST_TEST(robot2.id() == 10);
  BOOST_TEST(robot2.x() == 0.0);
  BOOST_TEST(robot2.y() == 0.0);
  BOOST_TEST(robot2.theta() == 0.0);

  // set value test
  robot2.x(40.0);
  BOOST_TEST(robot2.x() == 40.0);

  robot2.y(41.0);
  BOOST_TEST(robot2.y() == 41.0);

  robot2.vx(42.0);
  BOOST_TEST(robot2.vx() == 42.0);

  robot2.vy(43.0);
  BOOST_TEST(robot2.vy() == 43.0);

  robot2.theta(44.0);
  BOOST_TEST(robot2.theta() == 44.0);

  robot2.omega(45.0);
  BOOST_TEST(robot2.omega() == 45.0);
}

BOOST_AUTO_TEST_CASE(test003) {
  ai::model::robot robot3;

  BOOST_TEST(robot3.id() == 0);
  BOOST_TEST(robot3.x() == 0.0);
  BOOST_TEST(robot3.y() == 0.0);
  BOOST_TEST(robot3.theta() == 0.0);
}

BOOST_AUTO_TEST_SUITE_END()
