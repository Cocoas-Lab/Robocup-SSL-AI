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
  double ballSpeed   = std::hypot(ballVel.y(), ballVel.x());
  double ballAngle   = std::atan2(ballVel.y(), ballVel.x());

  // 自分のロボット、敵のロボット
  const auto friendRobots = isYellow_ ? world_.robotsYellow() : world_.robotsBlue();
  const auto enemyRobots  = isYellow_ ? world_.robotsBlue() : world_.robotsYellow();
  const auto robot        = util::math::position(friendRobots.at(id_));
  const auto robotTheta   = util::math::wrapToPi(friendRobots.at(id_).theta());

  const double robot2Ball = std::atan2(ball.y() - robot.y(), ball.x() - robot.x());

  // 扱いやすいように目標位置をベクトル形式に
  const auto target = util::math::position(target_);
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  auto theta = 0.0;

  const auto field = world_.field();

  if ((std::abs(robot.x()) > field.xMax() - field.penaltyLength() - 100.0) &&
      (std::abs(robot.y()) < field.penaltyWidth() / 2.0 + 100.0)) {
    // robot in the penalty area
    command.pos({0.0, 0.0, robotTheta});
  } else if ((std::abs(ballPos.x()) > field.xMax() - field.penaltyLength() - 100.0) &&
             (std::abs(ballPos.y()) < field.penaltyWidth() / 2.0 + 100.0)) {
    // ball in the penalty area
    command.vel({0.0, 0.0, 0.0});
  } else if (std::abs(ballPos.x()) > field.xMax() + 100.0 ||
             std::abs(ballPos.y()) > field.yMax() + 100.0) {
    // don't get the ball which is out of the field
    command.vel({0.0, 0.0, 0.0});
  } else if (false && (std::hypot(ballPos.x() - robot.x(), ballPos.y() - robot.y()) / 1000.0) *
                              (ballSpeed / 1000.0) * std::cos(ballAngle - robot2Ball) >
                          5.0) {
    // don't get the ball which is going out of the field (inner product)
    command.vel({0.0, 0.0, 0.0});
  } else {
    if (ballVel.norm() < 500) {
      auto radius = 140.0;
      // 目標に向かって蹴る
      {
        const auto theta =
            util::math::wrapToPi(std::atan2(target.y() - robot.y(), target.x() - robot.x()));
        // ± 6[deg]以内に目標位置が存在する場合
        if (std::abs(theta - robotTheta) < pi<double>() / 30) {
          command.dribble(0);
          radius = 70;

          // 敵の判定
          {
            constexpr auto radius = 1000;
            const auto tmp        = (robot - target).norm() / radius;
            const auto ratio      = 1 - tmp;

            decltype(robot) pos   = (-ratio * robot + target) / tmp;
            constexpr auto margin = 200.0;
            const auto shiftP     = util::math::calcIsoscelesVertexes(pos, robot, margin);
            polygon poly;
            bg::exterior_ring(poly) = boost::assign::list_of<point>(pos.x(), pos.y())(
                std::get<0>(shiftP).x(), std::get<0>(shiftP).y())(
                std::get<1>(shiftP).x(), std::get<1>(shiftP).y())(pos.x(), pos.y());
            bool enemy = false;
            //自分から1000の位置と自分で三角形を作り,間に敵が1つでもあったらチップにする
            for (auto it : enemyRobots) {
              const point p((it.second).x(), (it.second).y());
              if (!bg::disjoint(p, poly)) {
                enemy = true;
                break;
              }
            }
            if (enemy) {
              command.kick({model::command::kickType::Tip, 10});
            } else {
              command.kick({model::command::kickType::Straight, 10});
            }
          }
        } else {
          command.kick({model::command::kickType::None, 0});
          command.dribble(0);
        }
      }

      // 移動目標
      {
        const auto ratio =
            (target - ballPos).norm() == 0 ? 0 : radius / ((target - ballPos).norm() + radius);
        position = (-ratio * target + 1 * ballPos) / (1 - ratio);
      }

      // 回り込むかどうかとか
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
        theta =
            util::math::wrapToPi(std::atan2(target.y() - robot.y(), target.x() - robot.x()));
      }
    } else {
      // TODO:grSimでは検証が難しいので実機で検証する必要あり

      // ボールが早いので予測位置に回り込む
      // 内積で直交位置を導出
      {
        const auto normalized = ballVel.normalized();
        const auto dist       = robot - ballPos;
        const auto dot        = normalized.dot(dist);

        position = (ballPos + dot * normalized);
      }
      command.kick({model::command::kickType::None, 0});

      if (std::signbit(ballVel.dot(position - ballPos))) {
        position = ball;
        {
          const auto margin = 400.0;
          polygon poly;
          const auto tmp          = util::math::calcIsoscelesVertexes(robot, position, margin);
          bg::exterior_ring(poly) = boost::assign::list_of<point>(robot.x(), robot.y())(
              std::get<0>(tmp).x(), std::get<0>(tmp).y())(
              std::get<1>(tmp).x(), std::get<1>(tmp).y())(robot.x(), robot.y());
          const point p(ballPos.x(), ballPos.y());

          bool flag = false;
          if ((ballPos - robot).norm() < 2000) flag = true;
          //間にボールがあったら回り込む
          if (!bg::disjoint(p, poly) && flag) {
            //判定の幅
            const auto margin       = 600.0;
            const auto [tmp1, tmp2] = util::math::calcIsoscelesVertexes(robot, ballPos, margin);
            const auto b2r          = std::atan2((robot - ball).y(), (robot - ball).x());
            const auto b2p          = std::atan2((position - ball).y(), (position - ball).x());
            position                = util::math::wrapToPi(b2p - b2r) < 0 ? tmp1 : tmp2;

          } else {
            const auto nb2r = std::atan2((robot - ballPos).y(), (robot - ballPos).x());
            const auto nb2p = std::atan2((position - ball).y(), (position - ball).x());
            // 当たらないようにする
            if (std::abs(util::math::wrapToPi(nb2r - nb2p)) < 0.1) {
              const auto margin = 200.0;
              const auto [tmp1, tmp2] =
                  util::math::calcIsoscelesVertexes(robot, position, margin);
              const auto b2r = std::atan2((robot - ball).y(), (robot - ball).x());
              const auto b2p = std::atan2((position - ball).y(), (position - ball).x());
              position       = util::math::wrapToPi(b2p - b2r) < 0 ? tmp1 : tmp2;
            }
          }
        }
      }
      theta =
          util::math::wrapToPi(std::atan2(ballPos.y() - robot.y(), ballPos.x() - robot.x()));
    }
    std::vector<planner::rrt::obstacle> obstacles{};
    obstacles.reserve(enemyRobots.size() + friendRobots.size());
    for (auto&& [iy, rs] : {std::forward_as_tuple(false, enemyRobots),
                            std::forward_as_tuple(true, friendRobots)}) {
      for (auto&& [i, r] : rs) {
        if (iy != isYellow_ || i != id_) {
          obstacles.push_back({model::command::position{r.x(), r.y(), 0.0}, 300.0});
        }
      }
    }
    rrt_->obstacles(obstacles);
    rrt_->search(model::command::position{robot.x(), robot.y(), robotTheta},
                 model::command::position{position.x(), position.y(), theta});
    command.pos(rrt_->target());
  }
  return command;
}
} // namespace action
} // namespace game
} // namespace ai
