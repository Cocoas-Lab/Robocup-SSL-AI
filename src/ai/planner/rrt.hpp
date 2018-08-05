#ifndef AI_PLANNER_RRT_HPP_
#define AI_PLANNER_RRT_HPP_

#include <list>
#include <memory>
#include <queue>
#include <vector>
#include <stdint.h>

#include "ai/model/command.hpp"
#include "ai/model/world.hpp"
#include "ai/planner/base.hpp"

namespace ai {
namespace planner {

/// @class : rrt
/// @brief : Path planner using RRT*
class rrt final : public base {
public:
  explicit rrt(const model::world& _world);
  //~rrt_star();

  // 節点
  struct node {
    position position_;                 // 座標
    double cost_;                       // 親ノードまでに必要なコスト
    std::weak_ptr<struct node> parent_; // 親ノードを指すポインタ
  };

  // 障害物
  struct obstacle {
    position position_; // 座標
    double r_;          // 物体の半径
  };

  /// @brief :  障害物の任意指定
  /// @param :  obstacles 障害物(struct object)のvector
  void obstacles(const std::vector<obstacle>& _obstacles);

  /// @brief : 探索関数(木の生成)
  /// @param : _start 初期位置
  /// @param : _goal  目標位置
  /// @param : _searchNum  探索を行う回数
  /// @param : _maxBranchLength 伸ばす枝の最大距離
  /// @param : _margin  避けるときのマージン
  void search(const position _start, const position _goal, const uint32_t _searchNum = 100,
              const double _maxBranchLength = 300.0, const double _margin = 150.0);

private:
  const model::world& world_;
  std::vector<std::shared_ptr<struct node>> tree_; // 探索木(nodeの集まり)
  std::shared_ptr<node> nearestNode_;              // 次の目標節点
  std::vector<obstacle> defaultObstacles_; // 固定障害物,セットしなくても避ける
  std::vector<obstacle> additionalObstacles_; // 追加障害物,任意にセットして避ける
  std::vector<obstacle> allObstacles_;        // 障害物,上記2つの合算
  std::queue<position>
      priorityPoints_; // あるループで生成された最適なルート木，次ループで優先して探索

  /// @brief  障害物が存在するか
  /// @param  _start 初期位置
  /// @param  _goal  目標位置
  /// @param  _margin  避けるときのマージン
  bool obstructed(const position _start, const position _goal, const double _margin = 150.0);

  /// @brief  ペナルティエリア内に点があるか
  /// @param  _point 見たい座標
  /// @param  _margin  避けるときのマージン
  bool inPenalty(const position _point, const double _margin = 150.0);

  /// @brief  フィールド外に点があるか
  /// @param  _point 見たい座標
  /// @param  _margin  避けるときのマージン
  bool outField(const position _point, const double _margin = 150.0);
};

} // namespace planner
} // namespace ai

#endif // AI_SERVER_PLANNER_RRT_STAR_H
