#define BOOST_TEST_DYN_LINK

#include <string>
#include <boost/test/unit_test.hpp>

#include "ai/filter/base.hpp"

using namespace std::string_literals;
namespace filter = ai::filter;

BOOST_AUTO_TEST_SUITE(filter_base)

// 新しい値を受け取ったときに更新されるfilter.
// filter::base<任意の型, filter::timing::on_updated>を継承し,
// update()メンバ関数をoverrideすれば良い.
class testFilter1 : public filter::base<std::string, filter::timing::OnUpdated> {
  std::string suffix_;

public:
  // 必要があれば任意のコンストラクタを追加する
  testFilter1(const std::string& _suffix) : suffix_(_suffix) {}

  // filterの更新処理
  std::string update(const std::string& _value, ai::util::TimePointType) override {
    // 何らかの処理を行い, 結果を返す
    return _value + suffix_;
  }
};

BOOST_AUTO_TEST_CASE(on_updated) {
  ai::util::TimePointType t{};

  testFilter1 f{"er"};

  BOOST_TEST(f.update("C++", t) == "C++er"s);
  BOOST_TEST(f.update("Haskell", t) == "Haskeller"s);
}

// 値の更新を任意のタイミングで行うfilter.
// filter::base<任意の型, filter::timing::manual>を継承し,
// 更新を行うpublicメンバ関数を実装すれば良い.
class testFilter2 : public filter::base<std::string, filter::timing::Manual> {
  // 型が長いので型エイリアスを作っておくと良いかも
  using OwnType = filter::base<std::string, filter::timing::Manual>;

  std::string prefix_;

public:
  // コンストラクタで値を受け取る必要がなければ
  // using filter::base<std::string, filter::timing::manual>::base;
  // コンストラクタで値を受け取る必要がある場合は次のようにする
  testFilter2(OwnType::LastValueFuncType _lf, OwnType::WriterFuncType _wf,
              const std::string& _prefix)
      : base(_lf, _wf), prefix_(_prefix) {}

  // 更新を行うメンバ関数
  // 名前は適当で良い
  void addPrefix() {
    // 対象データの取得にはlastValue()を使う
    const auto value = lastValue();
    if (value) {
      // 値の更新はwrite()を使う
      write(prefix_ + *value);
    } else {
      // std::nulloptで更新すれば値が存在しないものとして処理される
      write(std::nullopt);
    }
  }
};

BOOST_AUTO_TEST_CASE(manual) {
  // 更新対象のデータ
  std::optional<std::string> value{"Haskell"};

  // 値を取得するための関数
  auto f1 = [&value] { return value; };
  // 値を更新するための関数
  auto f2 = [&value](std::optional<std::string> _newValue) { value = _newValue; };

  testFilter2 f{f1, f2, "すごい"};

  // filterを初期化するだけでは値は変化しない
  BOOST_TEST(*value == "Haskell");

  // filterの状態を更新すると値が変化する
  f.addPrefix();
  BOOST_TEST(*value == "すごいHaskell");
  f.addPrefix();
  BOOST_TEST(*value == "すごいすごいHaskell");
  f.addPrefix();
  BOOST_TEST(*value == "すごいすごいすごいHaskell");

  // nulloptでも問題ない
  value = std::nullopt;
  BOOST_CHECK_NO_THROW(f.addPrefix());

  // 関数が登録されていなくても例外で落ちない
  testFilter2 badFilter{{}, {}, ""};
  BOOST_CHECK_NO_THROW(badFilter.addPrefix());
}

BOOST_AUTO_TEST_SUITE_END()
