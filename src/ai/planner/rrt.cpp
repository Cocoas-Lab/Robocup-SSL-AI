#include <cmath>
#include <random>
#include "ai/planner/rrt.hpp"

namespace ai {
namespace planner {
rrt::rrt(const model::world& _world) : world_(_world) {
  // ペナルティエリアを障害物指定(正方形の外接円で近似)
  defaultObstacles_.push_back(obstacle{position{world_.field().xMax(), 0.0, 0.0},
                                       static_cast<double>(world_.field().penaltyLength())});
  defaultObstacles_.push_back(obstacle{position{world_.field().xMin(), 0.0, 0.0},
                                       static_cast<double>(world_.field().penaltyLength())});
}

bool rrt::obstructed(const position _start, const position _goal, const double _margin) {
  // 2点が完全に一致していたら
  if (_start.x == _goal.x && _start.y == _goal.y) {
    return false;
  }

  double a = _goal.y - _start.y;
  double b = -(_goal.x - _start.x);
  double c = -a * _start.x - b * _start.y;

  bool obstructed = false;
  for (const auto& it : allObstacles_) {
    // 障害物と直線の距離
    double dist = std::abs(a * it.position_.x + b * it.position_.y + c) / std::hypot(a, b);
    // 直線の方向ベクトル
    double vx = (_goal.x - _start.x) / std::hypot(_start.x - _goal.x, _start.y - _goal.y);
    double vy = (_goal.y - _start.y) / std::hypot(_start.x - _goal.x, _start.y - _goal.y);
    // 障害物判定のための内点
    double px1 = _start.x + vx * (_margin + it.r_);
    double py1 = _start.y + vy * (_margin + it.r_);
    double px2 = _goal.x + vx * (_margin + it.r_);
    double py2 = _goal.y + vy * (_margin + it.r_);

    // 線に近く，p1,p2の点の四角形内に障害物があったら
    if (dist < _margin + it.r_ && it.position_.x + it.r_ > std::min(px1, px2) &&
        it.position_.x - it.r_ < std::max(px1, px2) &&
        it.position_.y + it.r_ > std::min(py1, py2) &&
        it.position_.y - it.r_ < std::max(py1, py2)) {
      obstructed = true;
      break;
    }
  }
  return obstructed;
}

bool rrt::inPenalty(const position _point, const double _margin) {
  if (std::abs(_point.x) > (world_.field().xMax() - world_.field().penaltyLength() - _margin) &&
      std::abs(_point.y) < (world_.field().penaltyWidth() / 2.0 - _margin)) {
    return true;
  }
  return false;
}

bool rrt::outField(const position _point, const double _margin) {
  if (std::abs(_point.x) > world_.field().xMax() + _margin ||
      std::abs(_point.y) > world_.field().yMax() + _margin) {
    return true;
  }
  return false;
}

void rrt::obstacles(const std::vector<obstacle>& _obstacles) {
  allObstacles_.clear();
  allObstacles_ = _obstacles;
  std::copy(defaultObstacles_.begin(), defaultObstacles_.end(),
            std::back_inserter(allObstacles_));
}

void rrt::search(const position _start, const position _goal, const uint32_t _searchNum,
                 const double maxBranchLength, const double _margin) {
  obstacle nearObstacle;
  position minPos = {world_.field().xMin(), world_.field().yMin(), 0};
  position maxPos = {world_.field().xMax(), world_.field().yMax(), 0};

  bool inAvoidRange = false;
  for (auto&& it : allObstacles_) {
    if (std::hypot(_start.x - it.position_.x, _start.y - it.position_.y) < it.r_) {
      nearObstacle = it;
      inAvoidRange = true;
    }
  }
  if (inAvoidRange) { // 避ける範囲内に開始地点が存在,特別処理
    double angle =
        std::atan2(_start.y - nearObstacle.position_.y, _start.x - nearObstacle.position_.x);
    target_.x     = nearObstacle.position_.x + nearObstacle.r_ * std::cos(angle);
    target_.y     = nearObstacle.position_.y + nearObstacle.r_ * std::sin(angle);
    target_.theta = 0.0;
  } else if (inPenalty(_start, _margin) ||
             outField(_start, 200.0)) { // penalty内かfield外，特別処理
    target_ = {0.0, 0.0, 0.0};
  } else { // RRT*
    tree_.clear();
    // 初期ノード追加
    tree_.push_back(std::make_shared<node>(node{_start, 0.0, {}}));

    uint32_t searchCount = 0;
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_real_distribution<> randX(minPos.x - _margin, maxPos.x + _margin);
    std::uniform_real_distribution<> randY(minPos.y - _margin, maxPos.y + _margin);

    double minDistToGoal = 10000; // 目標位置までの最短距離(初期値は大きく取る)

    while (searchCount < _searchNum) {
      searchCount++;
      position searchPos; // 次探索点
      bool onObstacle;    // 障害物上は探索しない
      double minDist                = 100000.0;
      std::shared_ptr<node> minNode = nullptr;
      position minNewNode;
      position nextNode;
      do {
        do {
          onObstacle = false;
          if (priorityPoints_.size() == 0) {
            searchPos.x = randX(mt);
            searchPos.y = randX(mt);
          } else {
            searchPos = priorityPoints_.front();
            priorityPoints_.pop();
          }
          for (const auto it : allObstacles_) {
            if (std::hypot(searchPos.x - it.position_.x, searchPos.y - it.position_.y) <
                (it.r_ + _margin)) {
              onObstacle = true;
            }
          }
        } while (onObstacle);
        // ツリー中の全ノードから次探索点までの距離を算出，最近傍で障害物のないものを選ぶ
        for (const auto tree : tree_) {
          // 距離算出
          double dist =
              std::hypot(searchPos.x - tree->position_.x, searchPos.y - tree->position_.y);
          nextNode.x =
              std::min(dist, maxBranchLength) * (searchPos.x - tree->position_.x) / dist +
              tree->position_.x;
          nextNode.y =
              std::min(dist, maxBranchLength) * (searchPos.y - tree->position_.y) / dist +
              tree->position_.y;

          // 距離比較
          if (minDist > dist) {
            // 障害物検査
            if (!(obstructed(searchPos, nextNode))) {
              minDist    = dist;
              minNode    = tree;
              minNewNode = nextNode;
            }
          }
        }
      } while (minNode == nullptr);

      // 伸ばせるノードを見つけていたらtree追加
      if (minNode != nullptr) {
        std::shared_ptr<node> newNode = std::make_shared<node>(
            node{minNewNode, minNode->cost_ + std::min(minDist, maxBranchLength), minNode});

        tree_.push_back(newNode);
        // 新ノードからゴールまでの距離を計算
        double distBetGoalToNewNode =
            std::hypot(_goal.x - newNode->position_.x, _goal.y - newNode->position_.y);
        if (minDistToGoal > distBetGoalToNewNode) {
          minDistToGoal = distBetGoalToNewNode;
          nearestNode_  = newNode;
        }
        // これまでの道と新しいノードからの道を比較してコスト低ならノード再接続
        for (auto& it : tree_) {
          // 追加した物自身だったらやめる
          if (it != newNode) {
            // コストの比較
            double rewiredCost =
                newNode->cost_ + std::hypot(newNode->position_.x - it->position_.x,
                                            newNode->position_.y - it->position_.y);
            if (it->cost_ > rewiredCost) {
              // 障害物検査
              searchPos = it->position_;
              nextNode  = newNode->position_;
              if (!(obstructed(searchPos, nextNode))) {
                it->cost_   = rewiredCost;
                it->parent_ = newNode;
              }
            }
          }
        }
      }
    }
    auto node = nearestNode_;
    std::queue<position>().swap(priorityPoints_);
    priorityPoints_.push(node->position_);
    // 障害物を挟まないノードをショートカットして
    // スムースをかける
    while (!node->parent_.expired() && !node->parent_.lock()->parent_.expired() &&
           node->parent_.lock()->parent_.lock() != nullptr) {
      if (!obstructed(node->position_, node->parent_.lock()->parent_.lock()->position_,
                      100.0)) {
        node->parent_ = node->parent_.lock()->parent_.lock();
      } else {
        node = node->parent_.lock();
        priorityPoints_.push(node->position_);
        target_ = node->position_;
      }
    }
    target_ = node->position_;
  }
  target_.theta = _goal.theta;
}

} // namespace planner
} // namespace ai
