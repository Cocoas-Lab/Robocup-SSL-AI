#ifndef AI_TEST_UTIL_SLOT_TESTING_HELPER_HPP_
#define AI_TEST_UTIL_SLOT_TESTING_HELPER_HPP_

#include <future>
#include <tuple>
#include <boost/signals2/signal.hpp>

/// @class   slotTestingHelper
/// @brief   Boost.Signals2を使ったクラスのテストに使うヘルパクラス
/// @details このクラスを使うことで,
/// テストしたいSignalが発火した時に呼び出されるSlotの引数を簡単に取得できるようになる.
template <class... ResultTypes>
class slotTestingHelper {
public:
  /// @param f      Slotを登録するメンバ関数のポインタ
  /// @param object テストするクラスのオブジェクト
  template <class Func, class T>
  slotTestingHelper(Func&& _f, T& _object)
      : future_{promise_.get_future()},
        // Signalとの接続情報をconnection_に保持する (デストラクト時のSlotの切断に必要)
        connection_{(_object.*_f)(
            // テストしたいSignalに,
            // 渡された引数をslot_testing_helper::slotに転送するlambdaを接続
            [this](auto&&... args) { slot(std::forward<decltype(args)>(args)...); })} {}

  ~slotTestingHelper() {
    // Slotを切断する
    connection_.disconnect();
  }

  std::tuple<ResultTypes...> result() {
    // 登録したSlotが呼び出されるのを待つ
    future_.wait();
    return result_;
  }

private:
  template <class... Args>
  void slot(Args&&... _args) noexcept {
    // 渡された引数をresult_に格納
    result_ = std::forward_as_tuple(std::forward<Args>(_args)...);
    // promise_を処理完了状態にする
    promise_.set_value();
  }

  // 接続するlambda内にmove captureすればpromiseをデータメンバとして持つ必要はなさそうだが,
  // コピー不可な関数オブジェクトは渡すことができないっぽい(Boost 1.63.0で確認)ので仕方なく
  std::promise<void> promise_;

  std::future<void> future_;
  std::tuple<ResultTypes...> result_;
  boost::signals2::connection connection_;
};

#endif // AI_SERVER_TEST_UTIL_SLOT_TESTING_HELPER_H
