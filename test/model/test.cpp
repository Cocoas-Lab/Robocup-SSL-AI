#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai/model/ball.hpp"

BOOST_AUTO_TEST_SUITE(ball_datatype)

BOOST_AUTO_TEST_CASE(test01) {
  // constructor
  ai::model::ball ball{1.0, 2.0};

  // get value
  BOOST_TEST(ball.x() == 1.0);
  BOOST_TEST(ball.y() == 2.0);
}

BOOST_AUTO_TEST_CASE(test02) {
  ai::model::ball ball{};

  // init
  BOOST_TEST(ball.x() == 0.0);
  BOOST_TEST(ball.y() == 0.0);

  // set x
  ball.x(4.0);
  BOOST_TEST(ball.x() == 4.0);
  // set y
  ball.y(5.0);
  BOOST_TEST(ball.y() == 5.0);
  // set vx
  ball.vx(7.0);
  BOOST_TEST(ball.vx() == 7.0);
  // set vy
  ball.vy(8.0);
  BOOST_TEST(ball.vy() == 8.0);
  // set ax
  ball.ax(9.0);
  BOOST_TEST(ball.ax() == 9.0);
  // set ay
  ball.ay(10.0);
  BOOST_TEST(ball.ay() == 10.0);
}

BOOST_AUTO_TEST_SUITE_END()
