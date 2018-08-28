#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai/util/enumStr.hpp"

using namespace ai;
BOOST_AUTO_TEST_SUITE(enum_string_utils)
		enum class color{
			RED,
			BLUE,
			BLACK
		};
BOOST_AUTO_TEST_CASE(enumStr) {
		util::enumStr<color> test;

		// addKey()でキーを登録できる
		test.addKey(color::RED, "RED");

		// addKeys()ではキーを同時に複数個追加できる
		test.addKeys({
						{color::BLUE, "BLUE"},
						{color::BLACK, "BLACK"}
						});

		// 列挙定数を引数として呼び出すと文字列を返す
		BOOST_TEST(test(color::RED) == "RED");
		BOOST_TEST(test(color::BLUE) == "BLUE");
		BOOST_TEST(test(color::BLACK) == "BLACK");
}
BOOST_AUTO_TEST_SUITE_END()
