#ifndef AI_FILTER_BASE_HPP_
#define AI_FILTER_BASE_HPP_

#include <chrono>
#include <functional>
#include <optional>

#include "ai/util/time.hpp"

namespace ai {
namespace filter {

/// フィルタの状態を更新するタイミング
enum class timing {
  OnUpdated, // 対象データが更新されたとき
  Manual,    // 任意のタイミング
};

template <class T, timing Timing>
class base;

template <class T>
/// @brief 新しい値を受け取ったときに更新されるfilter
class base<T, timing::OnUpdated> {
public:
  virtual ~base() = default;

  /// @brief                  filterの状態を更新する
  /// @param _value            新しい値
  /// @param _time             valueが取得された時刻
  /// @return                 filterを通した値
  ///
  /// filterの状態を更新するためのメンバ関数.
  /// updater::worldでは, Visionから新しい値を受け取ったときに呼び出される.
  virtual T update(const T& _value, util::TimePointType _time) = 0;
};

template <class T>
/// @brief 値の更新を任意のタイミングで行うfilter
class base<T, timing::Manual> {
public:
  /// 最新の値を取得する関数オブジェクトの型
  using LastValueFuncType = std::function<std::optional<T>(void)>;
  /// 対象の値を更新する関数オブジェクトの型
  using WriterFuncType = std::function<void(std::optional<T>)>;

  /// @param _lastValueFunc  最新の値を取得する関数オブジェクト
  /// @param _writerFunc      対象の値を更新する関数オブジェクト
  base(LastValueFuncType _lastValueFunc, WriterFuncType _writerFunc)
      : lastValueFunc_(_lastValueFunc), WriterFunc_(_writerFunc) {}

  virtual ~base() = default;

protected:
  /// @brief                  最新の値を取得する
  /// @return                 最新の値
  ///
  /// lastValueFunc_に設定された関数を使い, 対象の最新の値を取得する.
  /// 対象が存在しなかったときなどはstd::nulloptを返す.
  /// (updater::robotでは, ロボットがロストしていたときstd::nulloptを返す)
  std::optional<T> lastValue() {
    if (lastValueFunc_) {
      return lastValueFunc_();
    } else {
      return std::nullopt;
    }
  }

  /// @brief                  対象の値を更新する
  /// @param _value            更新する値
  ///
  /// writerFunc_に設定された関数を使い, 対象の値をvalueで更新する.
  /// std::nulloptを渡した場合, 対象が存在しなかったとして扱われる.
  /// (updater::robotでは, ロボットがロストしていたとして処理される)
  void write(std::optional<T> _value) {
    if (writerFunc_) {
      writerFunc_(std::move(_value));
    }
  }

private:
  /// 最新の値を取得する関数オブジェクト
  LastValueFuncType lastValueFunc_;
  /// 対象の値を更新する関数オブジェクト
  writerFuncType writerFunc_;
};

} // namespace filter
} // namespace ai

#endif // AI_SERVER_FILTER_BASE_H
