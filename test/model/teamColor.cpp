#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai/model/teamColor.hpp"

BOOST_AUTO_TEST_SUITE(teamColor)

BOOST_AUTO_TEST_CASE(test001) {
	ai::model::teamColor color;

	color = ai::model::teamColor::Blue;
	BOOST_TEST(color == ai::model::teamColor::Blue);
}

BOOST_AUTO_TEST_SUITE_END();
