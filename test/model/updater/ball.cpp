#define BOOST_TEST_DYN_LINK

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai/filter/base.hpp"
#include "ai/model/updater/ball.hpp"
#include "ai/util/math/affine.hpp"

using namespace std::chrono_literals;

namespace filter = ai::filter;
namespace model  = ai::model;
namespace util   = ai::util;

BOOST_AUTO_TEST_SUITE(updater_ball)

BOOST_AUTO_TEST_CASE(normal) {
  model::updater::ball bu;

  {
    // デフォルトコンストラクが呼ばれたときに, 内部の値がちゃんと初期化されているか
    const auto b1 = bu.value();
    const auto b2 = model::ball{};
    BOOST_TEST(b1.x() == b2.x());
    BOOST_TEST(b1.y() == b2.y());
  }

  {
    ssl_protos::vision::DetectionFrame f;
    f.set_camera_id(0);

    auto b1 = f.add_balls();
    b1->set_x(1);
    b1->set_y(2);
    b1->set_z(3);
    b1->set_confidence(90.0);

    auto b2 = f.add_balls();
    b2->set_x(4);
    b2->set_y(5);
    b2->set_z(6);
    b2->set_confidence(92.0);

    bu.update(f);
  }

  {
    const auto b = bu.value();
    // confidenceの高い方の値が選択される
    BOOST_TEST(b.x() == 4);
    BOOST_TEST(b.y() == 5);
  }

  {
    ssl_protos::vision::DetectionFrame f;
    f.set_camera_id(1);

    auto b1 = f.add_balls();
    b1->set_x(10);
    b1->set_y(20);
    b1->set_z(30);
    b1->set_confidence(94.0);

    bu.update(f);
  }

  {
    const auto b = bu.value();
    // 違うカメラでよりconfidenceの高い方の値が検出されたらそちらが選択される
    BOOST_TEST(b.x() == 10);
    BOOST_TEST(b.y() == 20);
  }

  {
    ssl_protos::vision::DetectionFrame f;
    f.set_camera_id(2);

    auto b1 = f.add_balls();
    b1->set_x(100);
    b1->set_y(200);
    b1->set_z(300);
    b1->set_confidence(88.0);

    bu.update(f);
  }

  {
    const auto b = bu.value();
    // 違うカメラでconfidenceの低い値が検出されても値は変化しない
    BOOST_TEST(b.x() == 10);
    BOOST_TEST(b.y() == 20);
  }

  {
    ssl_protos::vision::DetectionFrame f;
    // 現在選択されている値が検出されたcam1の値を消す
    f.set_camera_id(1);
    bu.update(f);
  }

  {
    const auto b = bu.value();
    // cam0で検出されたball{4, 5, 6}が選ばれるが, 古い値では更新されない
    BOOST_TEST(b.x() == 10);
    BOOST_TEST(b.y() == 20);
  }

  {
    ssl_protos::vision::DetectionFrame f;
    // 候補リストを空にする
    f.set_camera_id(0);
    bu.update(f);
    f.set_camera_id(2);
    bu.update(f);
  }

  {
    const auto b = bu.value();
    // 候補リストが空になったら何もしない (最後に選択された値になる)
    BOOST_TEST(b.x() == 10);
    BOOST_TEST(b.y() == 20);
  }
}

BOOST_AUTO_TEST_CASE(transformation, *boost::unit_test::tolerance(0.0000001)) {
  using namespace boost::math::double_constants;

  model::updater::ball bu;
  // 90度回転, x軸方向に10, y軸方向に20平行移動
  bu.transformationMatrix(util::math::makeTransformationMatrix(10.0, 20.0, half_pi));

  {
    ssl_protos::vision::DetectionFrame f;
    f.set_camera_id(0);

    auto b1 = f.add_balls();
    b1->set_x(100);
    b1->set_y(200);
    b1->set_z(300);
    b1->set_confidence(90.0);

    bu.update(f);
  }

  {
    const auto b = bu.value();
    BOOST_TEST(b.x() == -190);
    BOOST_TEST(b.y() == 120.0);
  }
}

struct mockFilter1 : public filter::base<model::ball, filter::timing::OnUpdated> {
  // コンストラクタの引数に渡された値
  int arg1;
  int arg2;

  // 最後にupdateの引数に与えられた値
  model::ball v;
  util::TimePointType t;

  mockFilter1(int a1, int a2) : arg1(a1), arg2(a2) {}

  model::ball update(const model::ball& _value, util::TimePointType _time) override {
    v = _value;
    t = _time;

    // vx, ayに適当な値をセットして返す
    model::ball b{};
    b.vx(v.x() * 2);
    b.ay(v.y() * 3);
    return b;
  }
};

BOOST_AUTO_TEST_CASE(on_updated_filter) {
  model::updater::ball bu;

  // mock_filter1を設定
  const auto fp = bu.setFilter<mockFilter1>(123, 456).lock();

  // tをhigh_resolution_clockの型に変換する関数 (長すぎ)
  auto durationCast = [](auto t) { return std::chrono::duration_cast<util::DurationType>(t); };

  // set_filterの引数に与えた値がFilterのコンストラクタに正しく渡されているか
  BOOST_TEST(fp->arg1 == 123);
  BOOST_TEST(fp->arg2 == 456);

  {
    ssl_protos::vision::DetectionFrame f;
    f.set_camera_id(0);
    f.set_t_capture(2.0);

    auto b1 = f.add_balls();
    b1->set_x(1);
    b1->set_y(2);
    b1->set_z(3);
    b1->set_confidence(90.0);

    bu.update(f);
  }

  {
    // updateの引数に値が正しく渡されているか
    BOOST_TEST(fp->v.x() == 1);
    BOOST_TEST(fp->v.y() == 2);
    BOOST_TEST(fp->t.time_since_epoch().count() == durationCast(2s).count());

    // vx, ayが選択された値の2倍, 3倍になっている
    const auto b = bu.value();
    BOOST_TEST(b.vx() == 2);
    BOOST_TEST(b.ay() == 6);
  }

  {
    ssl_protos::vision::DetectionFrame f;
    f.set_camera_id(1);
    f.set_t_capture(4.0);

    auto b1 = f.add_balls();
    b1->set_x(10);
    b1->set_y(20);
    b1->set_z(30);
    b1->set_confidence(92.0);

    bu.update(f);
  }

  {
    // updateの引数に値が正しく渡されているか
    BOOST_TEST(fp->v.x() == 10);
    BOOST_TEST(fp->v.y() == 20);
    BOOST_TEST(fp->t.time_since_epoch().count() == durationCast(4s).count());

    // vx, ayが選択された値の2倍, 3倍になっている
    const auto b = bu.value();
    BOOST_TEST(b.vx() == 20);
    BOOST_TEST(b.ay() == 60);
  }

  {
    ssl_protos::vision::DetectionFrame f;
    f.set_camera_id(0);
    f.set_t_capture(8.0);

    auto b1 = f.add_balls();
    b1->set_x(100);
    b1->set_y(200);
    b1->set_z(300);
    b1->set_confidence(90.0);

    bu.update(f);
  }

  {
    // 最新でない値が選択された場合は変化しない
    const auto b = bu.value();
    BOOST_TEST(b.vx() == 20);
    BOOST_TEST(b.ay() == 60);
  }
}

struct mockFilter2 : public filter::base<model::ball, filter::timing::Manual> {
  using own_type = filter::base<model::ball, filter::timing::Manual>;

  // コンストラクタの引数に渡された値
  int arg1;
  int arg2;

  mockFilter2(own_type::LastValueFuncType _lf, own_type::WriterFuncType _wf, int _a1, int _a2)
      : base(_lf, _wf), arg1(_a1), arg2(_a2) {}

  auto lv() {
    return lastValue();
  }

  void wv(std::optional<model::ball> _v) {
    write(_v);
  }
};

BOOST_AUTO_TEST_CASE(manual_filter) {
  model::updater::ball bu;

  // mock_filter2を設定
  const auto fp = bu.setFilter<mockFilter2>(123, 456).lock();

  // set_filterの引数に与えた値がFilterのコンストラクタに正しく渡されているか
  BOOST_TEST(fp->arg1 == 123);
  BOOST_TEST(fp->arg2 == 456);

  // 初期状態はnullopt
  BOOST_TEST(!fp->lv());

  {
    ssl_protos::vision::DetectionFrame f;
    f.set_camera_id(0);
    f.set_t_capture(2.0);

    auto b1 = f.add_balls();
    b1->set_x(1);
    b1->set_y(2);
    b1->set_z(3);
    b1->set_confidence(90.0);

    bu.update(f);
  }

  {
    // updateが呼ばれただけでは値は更新されない
    const auto b1 = bu.value();
    const auto b2 = model::ball{};
    BOOST_TEST(b1.x() == b2.x());
    BOOST_TEST(b1.y() == b2.y());

    // last_valueから選択された値が取れる
    const auto lv = fp->lv();
    BOOST_TEST(static_cast<bool>(lv));
    BOOST_TEST(lv->x() == 1);
    BOOST_TEST(lv->y() == 2);
  }

  {
    // writeで値が更新される
    const auto b1 = model::ball{4, 5};
    fp->wv(b1);

    const auto b2 = bu.value();
    BOOST_TEST(b2.x() == b1.x());
    BOOST_TEST(b2.y() == b1.y());

    // nulloptで更新しても値は変化しない
    fp->wv(std::nullopt);
    const auto b3 = bu.value();
    BOOST_TEST(b3.x() == b1.x());
    BOOST_TEST(b3.y() == b1.y());
  }

  {
    // 候補リストを空にするとlast_valueがnulloptを返す
    ssl_protos::vision::DetectionFrame f;
    f.set_camera_id(0);
    bu.update(f);
    BOOST_TEST(!fp->lv());
  }
}

BOOST_AUTO_TEST_CASE(clear_filter) {
  model::updater::ball bu;

  // clear_filterを呼ぶと登録したFilterが死んでいる
  const auto fp1 = bu.setFilter<mockFilter1>(123, 456);
  BOOST_TEST(!fp1.expired());
  bu.clearFilter();
  BOOST_TEST(fp1.expired());

  // manualなFilterも同様
  const auto fp2 = bu.setFilter<mockFilter2>(123, 456);
  BOOST_TEST(!fp2.expired());
  bu.clearFilter();
  BOOST_TEST(fp2.expired());

  // on_updatedなFilterが登録されている状態でmanualなFilterを登録するとon_updatedなFilterは死ぬ
  const auto fp3 = bu.setFilter<mockFilter1>(123, 456);
  const auto fp4 = bu.setFilter<mockFilter2>(123, 456);
  BOOST_TEST(fp3.expired());
  BOOST_TEST(!fp4.expired());

  // 逆も同様
  const auto fp5 = bu.setFilter<mockFilter1>(123, 456);
  BOOST_TEST(fp4.expired());
  BOOST_TEST(!fp5.expired());
}

BOOST_AUTO_TEST_SUITE_END()
