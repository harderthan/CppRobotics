#include "a_star.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>
#include <utility>

#include "matplotlibcpp.h"

namespace path_planning {
namespace a_star {

namespace plt = matplotlibcpp;

AStarPlanner::AStarPlanner(double resolution, double robot_radius,
                           bool is_show_animation)
    : resolution_(resolution),
      robot_radius_(robot_radius),
      min_x_(std::numeric_limits<double>::max()),
      max_x_(std::numeric_limits<double>::min()),
      min_y_(std::numeric_limits<double>::max()),
      max_y_(std::numeric_limits<double>::min()),
      x_width_(0),
      y_width_(0),
      is_show_animation_(is_show_animation) {
  if (resolution_ <= 0.0) {
    std::cerr << "Resolution must be greater than 0." << std::endl;
    abort();
  }
  if (robot_radius_ < 0.0) {
    std::cerr << "Robot radius must be greater than or equal to 0."
              << std::endl;
    abort();
  }

  min_x_ = std::numeric_limits<double>::max();
  max_x_ = std::numeric_limits<double>::min();
  min_y_ = std::numeric_limits<double>::max();
  max_y_ = std::numeric_limits<double>::min();
  x_width_ = 0;
  y_width_ = 0;

  motions_ = {{1, 0, 1.0},   {0, 1, 1.0},    {-1, 0, 1.0},   {0, -1, 1.0},
              {1, 1, 1.414}, {1, -1, 1.414}, {-1, 1, 1.414}, {-1, -1, 1.414}};
}

std::vector<Position> AStarPlanner::Plan(const Position &start,
                                         const Position &goal) {
  if (obstacle_map.empty()) {
    std::cerr << "Obstacle positions must be set." << std::endl;
    abort();
  }
  if (start.x < min_x_ || start.x >= max_x_ || start.y < min_y_ ||
      start.y >= max_y_) {
    std::cerr << "Start position is out of bounds." << std::endl;
    abort();
  }
  if (goal.x < min_x_ || goal.x >= max_x_ || goal.y < min_y_ ||
      goal.y >= max_y_) {
    std::cerr << "Goal position is out of bounds." << std::endl;
    abort();
  }

  Node start_node = {0.0, CalculateXYIndex(start) % x_width_,
                     CalculateXYIndex(start) / x_width_, -1};
  Node goal_node = {0.0, CalculateXYIndex(goal) % x_width_,
                    CalculateXYIndex(goal) / x_width_, -1};

  std::unordered_map<int, Node> open_nodes;
  std::unordered_map<int, Node> closed_nodes;
  open_nodes[CalculateGridIndex(start_node)] = start_node;

  while (!open_nodes.empty()) {
    auto current_node_iter = std::min_element(
        open_nodes.begin(), open_nodes.end(),
        [&goal_node, this](const std::pair<int, Node> &lhs,
                           const std::pair<int, Node> &rhs) {
          return lhs.second.cost + CalculateHeuristic(lhs.second, goal_node) <
                 rhs.second.cost + CalculateHeuristic(rhs.second, goal_node);
        });

    if (is_show_animation_) {
      auto position =
          CalculateGridPosition(CalculateGridIndex(current_node_iter->second));
      plt::plot({position.x}, {position.y}, "xc");
      if (closed_nodes.size() % 10 == 0) {
        plt::pause(0.001);
      }
    }

    if (current_node_iter->second.index_x == goal_node.index_x &&
        current_node_iter->second.index_y == goal_node.index_y) {
      goal_node.parent_index = current_node_iter->second.parent_index;
      goal_node.cost = current_node_iter->second.cost;
      break;
    }

    closed_nodes[current_node_iter->first] = current_node_iter->second;
    open_nodes.erase(current_node_iter);

    for (const auto &motion : motions_) {
      auto node = Node{current_node_iter->second.cost + motion.cost,
                       current_node_iter->second.index_x + motion.dx,
                       current_node_iter->second.index_y + motion.dy,
                       CalculateGridIndex(current_node_iter->second)};
      if (!VerifyNode(node)) {
        continue;
      }
      if (closed_nodes.find(CalculateGridIndex(node)) != closed_nodes.end()) {
        continue;
      }
      auto it = open_nodes.find(CalculateGridIndex(node));
      if (it == open_nodes.end()) {
        open_nodes[CalculateGridIndex(node)] = node;
      } else if (it->second.cost > node.cost) {
        it->second = node;
      }
    }
  }

  auto path = CalculateFinalPath(goal_node, closed_nodes);
  return path;
}

void AStarPlanner::SetObstaclePositions(
    const std::vector<Position> &obstacle_positions) {
  if (obstacle_positions.empty()) {
    std::cerr << "Obstacle positions must not be empty." << std::endl;
    abort();
  }

  // Calculate the min and max x and y values
  for (const auto &p : obstacle_positions) {
    min_x_ = std::min(min_x_, p.x);
    max_x_ = std::max(max_x_, p.x);
    min_y_ = std::min(min_y_, p.y);
    max_y_ = std::max(max_y_, p.y);
  }
  std::cout << "min_x: " << min_x_ << "\n"
            << "max_x: " << max_x_ << "\n"
            << "min_y: " << min_y_ << "\n"
            << "max_y: " << max_y_ << std::endl;

  // Calculate the width of the x and y values
  x_width_ = static_cast<int>((max_x_ - min_x_) / resolution_);
  y_width_ = static_cast<int>((max_y_ - min_y_) / resolution_);
  std::cout << "x_width: " << x_width_ << "\n"
            << "y_width: " << y_width_ << std::endl;

  // Mark the obstacle positions on the obstacle map
  obstacle_map.clear();
  obstacle_map.resize(x_width_ * y_width_, false);
  for (const auto &obstacle : obstacle_positions) {
    for (int index = 0; index < obstacle_map.size(); index++) {
      auto position = CalculateGridPosition(index);
      if (std::hypot(position.x - obstacle.x, position.y - obstacle.y) <=
          robot_radius_) {
        obstacle_map[index] = true;
      }
    }
  }
}

std::vector<Position> AStarPlanner::CalculateFinalPath(
    const Node &goal_node,
    const std::unordered_map<int, Node> &closed_nodes) const {
  std::vector<Position> path;

  if (goal_node.parent_index == -1) {
    std::cerr << "Cannot find the goal node." << std::endl;
    return path;
  }

  auto current_node = goal_node;
  while (current_node.parent_index != -1) {
    path.push_back(CalculateGridPosition(CalculateGridIndex(current_node)));
    current_node = closed_nodes.at(current_node.parent_index);
  }

  return path;
}

Position AStarPlanner::CalculateGridPosition(int index) const {
  return {static_cast<double>(index % x_width_) * resolution_ + min_x_,
          static_cast<double>(index / x_width_) * resolution_ + min_y_};
}

double AStarPlanner::CalculateHeuristic(const Node &node,
                                        const Node &goal) const {
  return std::hypot(node.index_x - goal.index_x, node.index_y - goal.index_y);
}

int AStarPlanner::CalculateXYIndex(const Position &position) const {
  return static_cast<int>((position.y - min_y_) / resolution_) * x_width_ +
         static_cast<int>((position.x - min_x_) / resolution_);
}

int AStarPlanner::CalculateGridIndex(const Node &node) const {
  return (node.index_y - min_y_) * x_width_ + (node.index_x - min_x_);
}

bool AStarPlanner::VerifyNode(const Node &node) const {
  const auto position = CalculateGridPosition(CalculateGridIndex(node));
  if (position.x < min_x_ || position.x >= max_x_ || position.y < min_y_ ||
      position.y >= max_y_) {
    return false;
  }
  // Check collision.
  return !obstacle_map[CalculateXYIndex(position)];
}

}  // namespace a_star
}  // namespace path_planning
