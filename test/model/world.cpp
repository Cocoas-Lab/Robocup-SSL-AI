#define BOOST_TEST_DYN_LINK

#include <stdexcept>
#include <boost/test/unit_test.hpp>

#include "ai/model/world.hpp"

#include "ssl-protos/vision/detection.pb.h"

BOOST_AUTO_TEST_SUITE(world_model)

BOOST_AUTO_TEST_CASE(test001) {
  ai::model::world w;

  {
    // 保持するメンバが正しく初期化されているか

    ai::model::field f{};
    BOOST_TEST(w.field().length() == f.length());
    BOOST_TEST(w.field().width() == f.width());
    BOOST_TEST(w.field().centerRadius() == f.centerRadius());
    BOOST_TEST(w.field().goalWidth() == f.goalWidth());
    BOOST_TEST(w.field().penaltyLength() == f.penaltyLength());
    BOOST_TEST(w.field().penaltyWidth() == f.penaltyWidth());

    ai::model::ball b{};
    BOOST_TEST(w.ball().x() == b.x());
    BOOST_TEST(w.ball().y() == b.y());

    BOOST_TEST(w.robotsBlue().size() == 0);
    BOOST_TEST(w.robotsYellow().size() == 0);
  }

  {
    // getter/setterが正しく動作するか

    ai::model::field field{};
    field.length(1);
    w.field(field);

    ai::model::ball ball{123, 456};
    w.ball(ball);

    ai::model::world::RobotsList robotsBlue{{1, {1}}, {3, {3}}};
    w.robotsBlue(robotsBlue);

    ai::model::world::RobotsList robotsYellow{{2, {2}}, {4, {4}}};
    w.robotsYellow(robotsYellow);

    BOOST_TEST(w.field().length() == 1);
    BOOST_TEST(w.ball().x() = 123);
    BOOST_TEST(w.robotsBlue().size() == 2);
    BOOST_TEST(w.robotsBlue().count(1) == 1);
    BOOST_TEST(w.robotsBlue().count(3) == 1);
    BOOST_TEST(w.robotsYellow().size() == 2);
    BOOST_TEST(w.robotsYellow().count(2) == 1);
    BOOST_TEST(w.robotsYellow().count(4) == 1);

    ai::model::world w2{std::move(field), std::move(ball), std::move(robotsBlue),
                        std::move(robotsYellow)};
    BOOST_TEST(w2.field().length() == 1);
    BOOST_TEST(w2.ball().x() = 123);
    BOOST_TEST(w2.robotsBlue().size() == 2);
    BOOST_TEST(w2.robotsBlue().count(1) == 1);
    BOOST_TEST(w2.robotsBlue().count(3) == 1);
    BOOST_TEST(w2.robotsYellow().size() == 2);
    BOOST_TEST(w2.robotsYellow().count(2) == 1);
    BOOST_TEST(w2.robotsYellow().count(4) == 1);
  }

  {
    auto check = [](const auto& copy, const auto& original) {
      BOOST_TEST(copy.field().length() == original.field().length());
      BOOST_TEST(copy.ball().x() = original.ball().x());
      BOOST_TEST(copy.robotsBlue().size() == original.robotsBlue().size());
      BOOST_TEST(copy.robotsBlue().count(1) == original.robotsBlue().count(1));
      BOOST_TEST(copy.robotsBlue().count(3) == original.robotsBlue().count(3));
      BOOST_TEST(copy.robotsYellow().size() == original.robotsYellow().size());
      BOOST_TEST(copy.robotsYellow().count(1) == original.robotsYellow().count(1));
      BOOST_TEST(copy.robotsYellow().count(3) == original.robotsYellow().count(3));
    };

    // コピーコンストラクタが正しく動作するか
    ai::model::world w2(w);
    check(w2, w);

    // コピー代入演算子が正しく動作するか
    ai::model::world w3;
    w3 = w;
    check(w3, w);

    // コピーなのでwを変更してもw2, w3は変化しない
    const auto prevBall = w.ball();
    w.ball(ai::model::ball{12, 34});
    BOOST_TEST(w2.ball().x() = 123);
    BOOST_TEST(w3.ball().x() = 123);
    w.ball(prevBall);

    // ムーブコンストラクタが正しく動作するか
    ai::model::world w4(std::move(w3));
    check(w4, w);

    // ムーブ代入演算子が正しく動作するか
    ai::model::world w5;
    w5 = std::move(w4);
    check(w5, w);
  }
}

BOOST_AUTO_TEST_SUITE_END()
