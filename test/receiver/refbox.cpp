#define BOOST_TEST_DYN_LINK

#include <chrono>
#include <future>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "../util/slotTestingHelper.hpp"

#include "ai/receiver/refbox.hpp"
#include "ai/util/multicast/sender.hpp"

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace ai;
using namespace ai::receiver;
using namespace ai::util::multicast;

BOOST_TEST_DONT_PRINT_LOG_VALUE(ssl_protos::refbox::Referee::Stage)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ssl_protos::refbox::Referee::Command)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai::model::refbox::stageName)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ai::model::refbox::gameCommand)

BOOST_AUTO_TEST_SUITE(refbox_receiver)

BOOST_AUTO_TEST_CASE(send_and_receive, *boost::unit_test::timeout(30)) {
  // ダミーパケットの作成
  ssl_protos::refbox::Referee dummyFrame;

  ssl_protos::refbox::Referee_TeamInfo yellow;
  ssl_protos::refbox::Referee_TeamInfo blue;

  yellow.set_name("yellow");
  yellow.set_score(1);
  yellow.set_goalie(2);
  yellow.set_red_cards(3);
  yellow.set_yellow_cards(4);
  yellow.add_yellow_card_times(5);
  yellow.set_timeouts(6);
  yellow.set_timeout_time(7);
  blue.set_name("blue");
  blue.set_score(8);
  blue.set_goalie(9);
  blue.set_red_cards(10);
  blue.set_yellow_cards(11);
  blue.add_yellow_card_times(12);
  blue.set_timeouts(13);
  blue.set_timeout_time(14);

  dummyFrame.set_packet_timestamp(1);
  dummyFrame.set_stage(ssl_protos::refbox::Referee::Stage::Referee_Stage_NORMAL_FIRST_HALF_PRE);
  dummyFrame.set_stage_time_left(2);
  dummyFrame.set_command_counter(3);
  dummyFrame.set_command(ssl_protos::refbox::Referee::Command::Referee_Command_HALT);
  dummyFrame.set_command_timestamp(4);
  auto y = dummyFrame.mutable_yellow();
  y->CopyFrom(yellow);
  auto b = dummyFrame.mutable_blue();
  b->CopyFrom(blue);

  boost::asio::io_service ioService;

  // refbox受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.1, port = 10088
  refbox r(ioService, "0.0.0.0", "224.5.23.1", 10088);

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.1, port = 10088
  sender s(ioService, "224.5.23.1", 10088);

  // 受信を開始する
  std::thread t([&] { ioService.run(); });

  // 念の為少し待つ
  std::this_thread::sleep_for(50ms);

  {
    slotTestingHelper<ssl_protos::refbox::Referee> referee{&refbox::onReceive, r};

    // ダミーパケットを送信
    boost::asio::streambuf buf;
    std::ostream os(&buf);
    dummyFrame.SerializeToOstream(&os);
    s.send(buf.data());

    // 受信したデータを取得
    const auto f = std::get<0>(referee.result());

    // 受信したデータがダミーパケットと一致するか確認する

    BOOST_TEST(f.packet_timestamp() == dummyFrame.packet_timestamp());
    BOOST_TEST(f.stage() == dummyFrame.stage());
    BOOST_TEST(f.stage_time_left() == dummyFrame.stage_time_left());
    BOOST_TEST(f.command() == dummyFrame.command());
    BOOST_TEST(f.command_timestamp() == dummyFrame.command_timestamp());
    BOOST_TEST(f.yellow().name() == dummyFrame.yellow().name());
    BOOST_TEST(f.yellow().score() == dummyFrame.yellow().score());
    BOOST_TEST(f.yellow().goalie() == dummyFrame.yellow().goalie());
    BOOST_TEST(f.yellow().red_cards() == dummyFrame.yellow().red_cards());
    BOOST_TEST(f.yellow().yellow_cards() == dummyFrame.yellow().yellow_cards());
    BOOST_TEST(f.yellow().yellow_card_times(0) == dummyFrame.yellow().yellow_card_times(0));
    BOOST_TEST(f.yellow().timeouts() == dummyFrame.yellow().timeouts());
    BOOST_TEST(f.yellow().timeout_time() == dummyFrame.yellow().timeout_time());
    BOOST_TEST(f.blue().name() == dummyFrame.blue().name());
    BOOST_TEST(f.blue().score() == dummyFrame.blue().score());
    BOOST_TEST(f.blue().goalie() == dummyFrame.blue().goalie());
    BOOST_TEST(f.blue().red_cards() == dummyFrame.blue().red_cards());
    BOOST_TEST(f.blue().yellow_cards() == dummyFrame.blue().yellow_cards());
    BOOST_TEST(f.blue().yellow_card_times(0) == dummyFrame.blue().yellow_card_times(0));
    BOOST_TEST(f.blue().timeouts() == dummyFrame.blue().timeouts());
    BOOST_TEST(f.blue().timeout_time() == dummyFrame.blue().timeout_time());
  }

  // 受信の終了
  ioService.stop();
  t.join();
}

BOOST_AUTO_TEST_CASE(non_protobuf_data, *boost::unit_test::timeout(30)) {
  boost::asio::io_service ioService;

  // refbox受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.4, port = 10080
  refbox r(ioService, "0.0.0.0", "224.5.23.4", 10080);

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.4, port = 10080
  sender s(ioService, "224.5.23.4", 10080);

  // 受信を開始する
  std::thread t([&] { ioService.run(); });

  // 念の為少し待つ
  std::this_thread::sleep_for(50ms);

  {
    slotTestingHelper<> referee{&refbox::onError, r};

    // protobufじゃないデータを送信
    s.send("non protobuf data"s);

    // on_errorに設定したハンドラが呼ばれるまで待つ
    static_cast<void>(referee.result());
    BOOST_TEST(true);
  }

  // 受信の終了
  ioService.stop();
  t.join();
}

BOOST_AUTO_TEST_SUITE_END()
