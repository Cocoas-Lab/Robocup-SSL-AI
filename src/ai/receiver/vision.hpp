#ifndef AI_RECEIVER_VISION_HPP_
#define AI_RECEIVER_VISION_HPP_

#include <string>
#include <stdint.h>
#include <boost/asio.hpp>
#include <boost/signals2.hpp>

#include "ai/util/multicast/receiver.hpp"

#include "ssl-protos/vision/wrapper.pb.h"

namespace ai {
namespace receiver {

/// @class   vision
/// @brief   SSL-Visionからデータを受信するクラス
class vision {
  // データ受信時に呼ぶシグナルの型
  using ReceiveSignalType =
      boost::signals2::signal<void(const ssl_protos::vision::WrapperPacket&)>;
  // エラー時に呼ぶシグナルの型
  using ErrorSignalType = boost::signals2::signal<void(void)>;

public:
  /// @brief                  コンストラクタ
  /// @param listen_addr      通信に使うインターフェースのIPアドレス
  /// @param multicast_addr   マルチキャストアドレス
  /// @param port             ポート
  vision(boost::asio::io_service& _ioService, const std::string& _listenAddr,
         const std::string& _multicastAddr, uint16_t _port);

  /// @brief                  データ受信時に slot が呼ばれるようにする
  /// @param slot             データ受信時に呼びたい関数オブジェクト
  boost::signals2::connection onReceive(const ReceiveSignalType::slot_type& _slot);

  /// @brief                  エラー時に slot が呼ばれるようにする
  /// @param slot             エラー時に呼びたい関数オブジェクト
  boost::signals2::connection onError(const ErrorSignalType::slot_type& _slot);

private:
  void parsePacket(const util::multicast::receiver::Buffer& _buffer, std::size_t _size);

  ReceiveSignalType received_;
  ErrorSignalType errored_;

  util::multicast::receiver receiver_;
};

} // namespace receiver
} // namespace ai

#endif // AI_SERVER_RECEIVER_VISION_H
