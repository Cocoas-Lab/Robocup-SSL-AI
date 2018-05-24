#define BOOST_TEST_DYN_LINK

#include "ai/util/math/crc16.hpp"
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(crc16)

BOOST_AUTO_TEST_CASE(crc16_ibm){
	uint16_t crc = ai::util::math::crc16({0x55, 0x44});
	BOOST_TEST(crc == 0xFF90);
}

BOOST_AUTO_TEST_SUITE_END()
