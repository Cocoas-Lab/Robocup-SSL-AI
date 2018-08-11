#include <cmath>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/assign/list_of.hpp>

#include "getBall.hpp"
#include "ai/util/math/toVector.hpp"
#include "ai/util/math/angle.hpp"
#include "ai/util/math/geometry.hpp"


namespace ai {
namespace game {
namespace action {
void getBall::setTarget(const position _target) {
  target_ = _target;
  rrt_    = std::make_shared<ai::planner::rrt>(world_);
}

model::command getBall::execute() {
  namespace bg  = boost::geometry;
  using point   = bg::model::d2::point_xy<double>;
  using polygon = bg::model::polygon<point>;
  using boost::math::constants::pi;

  model::command command(id_);

  // ボールの位置、速度、未来位置
  const auto ballPos = util::math::position(world_.ball());
  const auto ballVel = util::math::velocity(world_.ball());
  const auto ball    = ballPos + ballVel * 3.0;

  // 自分のロボット、敵のロボット
  const auto friendRobots = isYellow_ ? world_.robotsYellow() : world_.robotsBlue();
  const auto enemyRobots  = isYellow_ ? world_.robotsBlue() : world_.robotsYellow();
  const auto robot        = util::math::position(friendRobots.at(id_));
  const auto robotTheta   = friendRobots.at(id_).theta();

  // 扱いやすいように目標位置をベクトル形式に
  const auto target = util::math::position(target_);
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  auto theta = 0.0;

  if (ballVel.norm() < 500) {
    auto radius = 140.0;
    // 目標に向かって蹴る
    {
      const auto theta =
          util::math::wrapToPi(std::atan2(target.y() - robot.y(), target.x() - robot.x()));
      // ± 6[deg]以内に目標位置が存在する場合
      if (std::abs(theta - robotTheta) < pi<double>() / 30) {
      }
    }

    // 移動目標
    {
      const auto ratio = radius / ((target - ballPos).norm() + radius);
      position         = (-ratio * target + 1 * ballPos) / (1 - ratio);
    }

    // 敵の判定
    {
      const auto margin = 120.0;
      polygon poly;
      const auto tmp          = util::math::calcIsoscelesVertexes(robot, position, margin);
      bg::exterior_ring(poly) = boost::assign::list_of<point>(robot.x(), robot.y())(
          std::get<0>(tmp).x(), std::get<0>(tmp).y())(
          std::get<1>(tmp).x(), std::get<1>(tmp).y())(robot.x(), robot.y());
      const point p(ballPos.x(), ballPos.y());

      if (!bg::disjoint(p, poly)) {
        //判定の幅
        const auto margin = 500.0;
        const auto tmp1 =
            std::get<0>(util::math::calcIsoscelesVertexes(robot, ballPos, margin));
        const auto tmp2 =
            std::get<1>(util::math::calcIsoscelesVertexes(robot, ballPos, margin));

        const auto tc = (ballPos.x() - target.x()) * (tmp1.y() - ballPos.y()) +
                        (ballPos.y() - target.y()) * (ballPos.x() - tmp1.x());
        const auto tp = (ballPos.x() - target.x()) * (robot.y() - ballPos.y()) +
                        (ballPos.y() - target.y()) * (ballPos.x() - robot.x());
        position = std::signbit(tc) == std::signbit(tp) ? tmp1 : tmp2;
      }
      theta = util::math::wrapToPi(std::atan2(target.y() - robot.y(), target.x() - robot.x()));
    }
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

  obstacles.push_back({model::command::position{ballPos.x(), ballPos.y(), 0.0}, 10.0});
  rrt_->obstacles(obstacles);
  rrt_->search(model::command::position{robot.x(), robot.y(), robotTheta},
               model::command::position{position.x(), position.y(), theta});
  command.pos(rrt_->target());

  return command;
}
} // namespace action
} // namespace game
} // namespace ai
