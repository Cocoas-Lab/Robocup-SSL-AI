#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai/model/teamInfo.hpp"

BOOST_AUTO_TEST_SUITE(teamInfo)

BOOST_AUTO_TEST_CASE(test001) {
  ai::model::teamInfo info("team name");

  BOOST_TEST(info.name() == "team name");
  BOOST_TEST(info.score() == 0);
  BOOST_TEST(info.goalie() == 0);
  BOOST_TEST(info.redCards() == 0);
  BOOST_TEST(info.yellowCards() == 0);
  BOOST_TEST(info.yellowCardTimes() == 0);
  BOOST_TEST(info.timeouts() == 0);
  BOOST_TEST(info.timeoutTimes() == 0);
}

BOOST_AUTO_TEST_CASE(test002) {
  ai::model::teamInfo info("nyan");

  info.score(2);
  info.goalie(10);
  info.redCards(3);
  info.yellowCards(8);
  info.yellowCardTimes(113);
  info.timeouts(2);
  info.timeoutTimes(255);

  BOOST_TEST(info.name() == "nyan");
  BOOST_TEST(info.goalie() == 10);
  BOOST_TEST(info.redCards() == 3);
  BOOST_TEST(info.yellowCards() == 8);
  BOOST_TEST(info.yellowCardTimes() == 113);
  BOOST_TEST(info.timeouts() == 2);
  BOOST_TEST(info.timeoutTimes() == 255);
}

BOOST_AUTO_TEST_SUITE_END()
