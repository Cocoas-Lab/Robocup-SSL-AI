#define BOOST_TEST_DYN_LINK

#include <chrono>
#include <future>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "../util/slotTestingHelper.hpp"

#include "ai/receiver/vision.hpp"
#include "ai/util/multicast/sender.hpp"

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace ai::receiver;
using namespace ai::util::multicast;

BOOST_AUTO_TEST_SUITE(vision_receiver)

BOOST_AUTO_TEST_CASE(send_and_receive, *boost::unit_test::timeout(30)) {
  // ダミーパケットの作成
  ssl_protos::vision::DetectionFrame dummyFrame;
  dummyFrame.set_frame_number(1);
  dummyFrame.set_t_capture(2.0);
  dummyFrame.set_t_sent(3.0);
  dummyFrame.set_camera_id(4);

  ssl_protos::vision::WrapperPacket dummyPacket;
  auto md = dummyPacket.mutable_detection();
  md->CopyFrom(dummyFrame);

  boost::asio::io_service ioService;

  // SSL-Vision受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.3, port = 10008
  vision v(ioService, "0.0.0.0", "224.5.23.3", 10008);

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.3, port = 10008
  sender s(ioService, "224.5.23.3", 10008);

  // 受信を開始する
  std::thread t([&] { ioService.run(); });

  // 念の為少し待つ
  std::this_thread::sleep_for(50ms);

  {
    slotTestingHelper<ssl_protos::vision::WrapperPacket> wrapper{&vision::onReceive, v};

    // ダミーパケットを送信
    boost::asio::streambuf buf;
    std::ostream os(&buf);
    dummyPacket.SerializeToOstream(&os);
    s.send(buf.data());

    // 受信したデータを取得
    const auto f = std::get<0>(wrapper.result());

    // 受信したデータがダミーパケットと一致するか確認する
    BOOST_TEST(f.has_detection());
    BOOST_TEST(!f.has_geometry());

    const auto& d = f.detection();
    BOOST_TEST(d.frame_number() == dummyFrame.frame_number());
    BOOST_TEST(d.t_capture() == dummyFrame.t_capture());
    BOOST_TEST(d.t_sent() == dummyFrame.t_sent());
    BOOST_TEST(d.camera_id() == dummyFrame.camera_id());
  }

  // 受信の終了
  ioService.stop();
  t.join();
}

BOOST_AUTO_TEST_CASE(non_protobuf_data, *boost::unit_test::timeout(30)) {
  boost::asio::io_service ioService;

  // SSL-Vision受信クラスの初期化
  // listen_addr = 0.0.0.0, multicast_addr = 224.5.23.4, port = 10010
  vision v(ioService, "0.0.0.0", "224.5.23.4", 10010);

  // 送信クラスの初期化
  // multicast_addr = 224.5.23.4, port = 10010
  sender s(ioService, "224.5.23.4", 10010);

  // 受信を開始する
  std::thread t([&] { ioService.run(); });

  // 念の為少し待つ
  std::this_thread::sleep_for(50ms);

  {
    slotTestingHelper<> wrapper{&vision::onError, v};

    // protobufじゃないデータを送信
    s.send("non protobuf data"s);

    // on_errorに設定したハンドラが呼ばれるまで待つ
    static_cast<void>(wrapper.result());
    BOOST_TEST(true);
  }

  // 受信の終了
  ioService.stop();
  t.join();
}

BOOST_AUTO_TEST_SUITE_END()
