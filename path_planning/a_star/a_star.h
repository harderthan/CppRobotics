#ifndef PATH_PLANNING_A_STAR_A_STAR_H_
#define PATH_PLANNING_A_STAR_A_STAR_H_

#include <sstream>
#include <unordered_map>
#include <vector>

namespace path_planning {
namespace a_star {

struct Position {
  double x;
  double y;
};

class AStarPlanner {
 public:
  AStarPlanner(const AStarPlanner &) = delete;
  AStarPlanner &operator=(const AStarPlanner &) = delete;
  AStarPlanner(AStarPlanner &&) = delete;
  AStarPlanner &operator=(AStarPlanner &&) = delete;
  ~AStarPlanner() = default;

  explicit AStarPlanner(double resolution, double robot_radius,
                        bool is_show_animation = true);

  std::vector<Position> Plan(const Position &start, const Position &goal);
  void SetObstaclePositions(const std::vector<Position> &obstacle_positions);

 private:
  struct Motion {
    int dx;
    int dy;
    double cost;
  };

  struct Node {
    double cost;
    int index_x;
    int index_y;
    int parent_index;

    std::ostream &operator<<(std::ostream &os) const {
      return os << "Node(" << cost << ", " << index_x << ", " << index_y << ", "
                << parent_index << ")";
    }
  };

  std::vector<Motion> GenerateMotions() const;
  std::vector<Position> CalculateFinalPath(
      const Node &goal_node,
      const std::unordered_map<int, Node> &closed_nodes) const;
  Position CalculateGridPosition(int index) const;
  double CalculateHeuristic(const Node &node, const Node &goal) const;
  int CalculateXYIndex(const Position &position) const;
  int CalculateGridIndex(const Node &node) const;
  bool VerifyNode(const Node &node) const;

  std::vector<Motion> motions_;
  std::vector<bool> obstacle_map;
  double resolution_;
  double robot_radius_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  int x_width_;
  int y_width_;
  bool is_show_animation_;
};

}  // namespace a_star
}  // namespace path_planning

#endif  // PATH_PLANNING_A_STAR_A_STAR_H_
