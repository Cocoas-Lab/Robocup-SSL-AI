#define BOOST_TEST_DYN_LINK

#include <chrono>
#include <memory>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "ai/controller/base.hpp"
#include "ai/driver.hpp"
#include "ai/model/teamColor.hpp"
#include "ai/model/updater/world.hpp"

using namespace std::chrono_literals;
namespace controller = ai::controller;
namespace model      = ai::model;

BOOST_TEST_DONT_PRINT_LOG_VALUE(model::teamColor)

BOOST_AUTO_TEST_SUITE(driver)

BOOST_AUTO_TEST_CASE(setTeamColor) {
  boost::asio::io_service ioService{};
  ai::model::updater::world wu{};

  {
    // コンストラクタでチームカラーが正しく設定されているか
    ai::driver d{ioService, 1s, wu, model::teamColor::Blue};
    BOOST_TEST(d.teamColor() == model::teamColor::Blue);

    // チームカラーが変更できるか
    d.teamColor(model::teamColor::Yellow);
    BOOST_TEST(d.teamColor() == model::teamColor::Yellow);
  }

  {
    ai::driver d{ioService, 1s, wu, model::teamColor::Yellow};
    BOOST_TEST(d.teamColor() == model::teamColor::Yellow);

    d.teamColor(model::teamColor::Blue);
    BOOST_TEST(d.teamColor() == model::teamColor::Blue);
  }
}

BOOST_AUTO_TEST_CASE(registerRobot) {
  boost::asio::io_service ioService{};
  ai::model::updater::world wu{};
  ai::driver d{ioService, std::chrono::seconds{1}, wu, model::teamColor::Yellow};

  ai::model::command cmd0{0};
  ai::model::command cmd1{1};

  // ロボットが登録されていない状態でupdate_command()を呼ぶとエラー
  BOOST_CHECK_THROW(d.updateCommand(cmd0), std::runtime_error);
  BOOST_CHECK_THROW(d.updateCommand(cmd1), std::runtime_error);

  // ID0のロボットを登録
  BOOST_CHECK_NO_THROW(d.registerRobot(0, nullptr, nullptr));
  BOOST_CHECK_NO_THROW(d.updateCommand(cmd0));

  // ID1のロボットを登録
  BOOST_CHECK_NO_THROW(d.registerRobot(1, nullptr, nullptr));
  BOOST_CHECK_NO_THROW(d.updateCommand(cmd1));

  // 登録したロボットが正常に削除されるか
  d.unregisterRobot(0);
  BOOST_CHECK_THROW(d.updateCommand(cmd0), std::runtime_error);
  BOOST_CHECK_NO_THROW(d.updateCommand(cmd1));
  d.unregisterRobot(1);
  BOOST_CHECK_THROW(d.updateCommand(cmd1), std::runtime_error);
}

struct mockController : public controller::base {
protected:
  controller::velocity update(const model::robot&, const controller::position&) {
    return {};
  }
  controller::velocity update(const model::robot&, const controller::velocity&) {
    return {};
  }
};

BOOST_AUTO_TEST_CASE(velocityLimit) {
  boost::asio::io_service ioService{};
  ai::model::updater::world wu{};
  ai::driver d{ioService, std::chrono::seconds{1}, wu, model::teamColor::Blue};

  // 適当なControllerをいくつか初期化し登録する
  // 後からアクセスできるように参照を残しておく
  auto c1Ptr     = std::make_unique<mockController>();
  auto c2Ptr     = std::make_unique<mockController>();
  const auto& c1 = *c1Ptr;
  const auto& c2 = *c2Ptr;
  d.registerRobot(1, std::move(c1Ptr), nullptr);
  d.registerRobot(2, std::move(c2Ptr), nullptr);

  // 変更前
  BOOST_TEST(c1.velocityLimit() == std::numeric_limits<double>::max());
  BOOST_TEST(c2.velocityLimit() == std::numeric_limits<double>::max());

  // 変更してみる
  d.velocityLimit(123);
  BOOST_TEST(c1.velocityLimit() == 123);
  BOOST_TEST(c2.velocityLimit() == 123);

  // さらに変更してみる
  d.velocityLimit(456);
  BOOST_TEST(c1.velocityLimit() == 456);
  BOOST_TEST(c2.velocityLimit() == 456);

  // もとに戻してみる
  d.velocityLimit(std::numeric_limits<double>::max());
  BOOST_TEST(c1.velocityLimit() == std::numeric_limits<double>::max());
  BOOST_TEST(c2.velocityLimit() == std::numeric_limits<double>::max());
}

BOOST_AUTO_TEST_SUITE_END()
