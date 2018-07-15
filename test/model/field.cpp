#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include "ai/model/field.hpp"

BOOST_AUTO_TEST_SUITE(field)

BOOST_AUTO_TEST_CASE(test001) {
  // constructor test
  ai::model::field field;

  BOOST_TEST(field.length() = 12000);
  BOOST_TEST(field.width() = 9000);
  BOOST_TEST(field.centerRadius() = 500);
  BOOST_TEST(field.goalWidth() = 1200);
  BOOST_TEST(field.penaltyLength() = 1200);
  BOOST_TEST(field.penaltyWidth() = 2400);
  BOOST_TEST(field.xMax() = 6000);
  BOOST_TEST(field.xMin() = -6000);
  BOOST_TEST(field.yMax() = 4500);
  BOOST_TEST(field.yMin() = -4500);
}

BOOST_AUTO_TEST_CASE(test002) {
  // setter test
  ai::model::field field;

  field.length(300);
  field.width(500);
  field.centerRadius(600);
  field.goalWidth(200);
  field.penaltyLength(10);
  field.penaltyWidth(30);

  BOOST_TEST(field.length() = 300);
  BOOST_TEST(field.width() = 500);
  BOOST_TEST(field.centerRadius() = 600);
  BOOST_TEST(field.goalWidth() = 200);
  BOOST_TEST(field.penaltyLength() = 10);
  BOOST_TEST(field.penaltyWidth() = 30);
  BOOST_TEST(field.xMax() = 150);
  BOOST_TEST(field.xMin() = -150);
  BOOST_TEST(field.yMax() = 250);
  BOOST_TEST(field.yMin() = -250);
}

BOOST_AUTO_TEST_SUITE_END()
