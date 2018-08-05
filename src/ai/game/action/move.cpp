#include <cmath>
#include <boost/math/constants/constants.hpp>
#include "move.hpp"

using boost::math::constants::pi;

namespace ai {
namespace game {
namespace action {

void move::moveTo(double _x, double _y, double _theta) {
  x_     = _x;
  y_     = _y;
  theta_ = _theta;
  rrt_   = std::make_shared<ai::planner::rrt>(world_);
}

model::command move::execute() {
  const double xyAllow = 15.0; //指定位置と取得した位置のズレの許容値[mm]
  const double thetaAllow =
      1.0 * pi<double>() / 180.0; //指定角度と取得した角度のズレの許容値[rad]
  const auto thisRobotTeam = isYellow_ ? world_.robotsYellow() : world_.robotsBlue();
  const auto& thisRobot    = thisRobotTeam.at(id_);
  model::command command(id_);

  if (std::abs(thisRobot.x() - x_) <= xyAllow && std::abs(thisRobot.y() - y_) <= xyAllow &&
      std::abs(thisRobot.theta() - theta_) <= thetaAllow) {
    //ロボットが指定位置に存在するとき
    finished_ = true;
  } else {
    //ロボットが指定位置に存在しないとき
    finished_ = false;
    // rrt_starによる経路生成
    const auto robotsBlue   = world_.robotsBlue();
    const auto robotsYellow = world_.robotsYellow();
    const auto ball         = world_.ball();

    std::vector<planner::rrt::obstacle> obstacles{};
    obstacles.reserve(robotsBlue.size() + robotsYellow.size());
    for (auto&& [iy, rs] : {std::forward_as_tuple(false, robotsBlue),
                            std::forward_as_tuple(true, robotsYellow)}) {
      for (auto&& [i, r] : rs) {
        if (iy != isYellow_ || i != id_) {
          obstacles.push_back({model::command::position{r.x(), r.y(), 0.0}, 300.0});
        }
      }
    }

    obstacles.push_back({model::command::position{ball.x(), ball.y(), 0.0}, 500.0});
    rrt_->obstacles(obstacles);
    rrt_->search(model::command::position{thisRobot.x(), thisRobot.y(), thisRobot.theta()},
                 model::command::position{x_, y_, theta_});
    command.pos(rrt_->target());
  }
  return command;
}
} // namespace action
} // namespace game
} // namespace ai
