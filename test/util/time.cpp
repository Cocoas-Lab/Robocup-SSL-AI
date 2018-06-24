#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai/util/time.hpp"

using namespace ai;
using namespace std::chrono_literals;

BOOST_AUTO_TEST_SUITE(time_utils)

BOOST_AUTO_TEST_CASE(toDuration) {
  // メタ関数のテスト
  static_assert(!util::decision::isDurationValue<int>, "");
  static_assert(util::decision::isDurationValue<std::chrono::duration<double>>, "");
  static_assert(util::decision::isDurationValue<std::chrono::microseconds>, "");

  // コンパイル時
  constexpr auto t1 = util::toDuration(1.234567);
  static_assert(t1 == std::chrono::duration_cast<std::chrono::microseconds>(1s + 234ms + 567us),
                "");
  static_assert(t1.count() == 1234567, "");

  // 実行時
  const double t2 = 5.678910;
  BOOST_TEST(util::toDuration(t2).count() == 5678910);
}

BOOST_AUTO_TEST_SUITE_END()
