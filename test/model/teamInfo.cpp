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

BOOST_AUTO_TEST_SUITE_END()
