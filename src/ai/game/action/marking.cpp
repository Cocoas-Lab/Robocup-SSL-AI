#include <cmath>
#include <Eigen/Dense>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/assign/list_of.hpp>

#include "marking.hpp"
#include "ai/util/math/angle.hpp"

namespace ai {
namespace game {
namespace action {
void marking::mark(uint32_t _id) {
  enemyId_ = _id;
}

void marking::mode(markMode _mode) {
  mode_ = _mode;
}

model::command marking::execute() {
  namespace bg  = boost::geometry;
  using point   = bg::model::d2::point_xy<double>;
  using polygon = bg::model::polygon<point>;
  using boost::math::constants::pi;

  model::command command(id_);

  // 各チームのロボット情報を取得
  const auto friendRobots = isYellow_ ? world_.robotsYellow() : world_.robotsBlue();
  const auto enemyRobots  = isYellow_ ? world_.robotsBlue() : world_.robotsYellow();

  // 指定したロボットが存在しない場合、ロボットを停止させる
  if (!enemyRobots.count(enemyId_) || !friendRobots.count(id_)) {
    command.vel({0, 0, 0});
    return command;
  }

  // ロボットを生成
  const auto myRobot    = friendRobots.at(id_);
  const auto enemyRobot = enemyRobots.at(enemyId_);
  const auto robotTheta = util::math::wrapToPi(friendRobots.at(id_).theta());

  // 扱いやすいようにベクトル表現に変換
  const Eigen::Vector2d enemy{enemyRobot.x(), enemyRobot.y()};
  const Eigen::Vector2d my{myRobot.x(), myRobot.y()};
  const Eigen::Vector2d ball{world_.ball().x(), world_.ball().y()};
  const Eigen::Vector2d goal{world_.field().xMin(), 0.0};
  Eigen::Vector2d tmpPos{0.0, 0.0};
  Eigen::Vector2d position{0.0, 0.0};
  auto ratio        = 0.0; //敵位置とボールの比
  auto tmp          = 0.0;
  const auto radius = 250;

  switch (mode_) {
    case PassCut:
      tmp      = (enemy - ball).norm() / radius; //敵位置 - 自位置の比
      ratio    = 1 - tmp;
      position = (-ratio * enemy + ball) / tmp;
      tmpPos   = ball;
      break;
  }

  //向きをボールの方へ
  const auto theta = util::math::wrapTo2pi(
      std::atan2(position.y() - ball.y(), position.x() - ball.x()) + pi<double>());

  //目標位置が外側に行ったらその場で停止
  if (std::abs(position.x()) > world_.field().xMax() ||
      std::abs(position.y()) > world_.field().yMax()) {
    command.vel({0.0, 0.0, 0.0});
    return command;
  }
  std::vector<planner::rrt::obstacle> obstacles{};
  obstacles.reserve(enemyRobots.size() + friendRobots.size());
  for (auto&& [iy, rs] :
       {std::forward_as_tuple(false, enemyRobots), std::forward_as_tuple(true, friendRobots)}) {
    for (auto&& [i, r] : rs) {
      if (iy != isYellow_ || i != id_) {
        obstacles.push_back({model::command::position{r.x(), r.y(), 0.0}, 300.0});
      }
    }
  }
  rrt_->obstacles(obstacles);
  rrt_->search(model::command::position{my.x(), my.y(), robotTheta},
               model::command::position{position.x(), position.y(), theta});
  command.pos(rrt_->target());
}
} // namespace action
} // namespace game
} // namespace ai
