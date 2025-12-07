#include "a_star.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

namespace path_planning {
namespace grid_based_search {
namespace a_star {
namespace {

struct Node {
  int x;
  int y;
  double cost;
  int parent_index;
};

struct Config {
  double resolution;
  double robot_radius;
  int min_x;
  int min_y;
  int max_x;
  int max_y;
  int x_width;
  int y_width;
  std::vector<bool> obstacle_map;
};

struct Motion {
  int dx;
  int dy;
  double cost;
};

double CalcGridPosition(int index, int min_position, double resolution) {
  return index * resolution + static_cast<double>(min_position);
}

int CalcXYIndex(double position, int min_position, double resolution) {
  return static_cast<int>(
      std::round((position - static_cast<double>(min_position)) / resolution));
}

int CalcGridIndex(const Node& node, const Config& config) {
  return (node.y - config.min_y) * config.x_width + (node.x - config.min_x);
}

double CalcHeuristic(const Node& n1, const Node& n2) {
  constexpr double kWeight = 1.0;
  return kWeight * std::hypot(static_cast<double>(n1.x - n2.x),
                              static_cast<double>(n1.y - n2.y));
}

std::vector<Motion> GetMotionModel() {
  return {{1, 0, 1.0},
          {0, 1, 1.0},
          {-1, 0, 1.0},
          {0, -1, 1.0},
          {-1, -1, std::sqrt(2.0)},
          {-1, 1, std::sqrt(2.0)},
          {1, -1, std::sqrt(2.0)},
          {1, 1, std::sqrt(2.0)}};
}

Config BuildConfig(const std::vector<double>& ox, const std::vector<double>& oy,
                   double resolution, double robot_radius) {
  Config config{};
  config.resolution = resolution;
  config.robot_radius = robot_radius;
  config.min_x =
      static_cast<int>(std::round(*std::min_element(ox.begin(), ox.end())));
  config.min_y =
      static_cast<int>(std::round(*std::min_element(oy.begin(), oy.end())));
  config.max_x =
      static_cast<int>(std::round(*std::max_element(ox.begin(), ox.end())));
  config.max_y =
      static_cast<int>(std::round(*std::max_element(oy.begin(), oy.end())));
  config.x_width = static_cast<int>(
      std::round((config.max_x - config.min_x) / config.resolution));
  config.y_width = static_cast<int>(
      std::round((config.max_y - config.min_y) / config.resolution));

  config.obstacle_map.assign(config.x_width * config.y_width, false);

  for (int ix = 0; ix < config.x_width; ++ix) {
    const double x = CalcGridPosition(ix, config.min_x, config.resolution);
    for (int iy = 0; iy < config.y_width; ++iy) {
      const double y = CalcGridPosition(iy, config.min_y, config.resolution);
      for (size_t i = 0; i < ox.size(); ++i) {
        const double distance = std::hypot(ox[i] - x, oy[i] - y);
        if (distance <= config.robot_radius) {
          config.obstacle_map[ix * config.y_width + iy] = true;
          break;
        }
      }
    }
  }

  return config;
}

bool VerifyNode(const Node& node, const Config& config) {
  const double px = CalcGridPosition(node.x, config.min_x, config.resolution);
  const double py = CalcGridPosition(node.y, config.min_y, config.resolution);

  if (px < config.min_x || py < config.min_y || px >= config.max_x ||
      py >= config.max_y) {
    return false;
  }

  const size_t map_index =
      static_cast<size_t>(node.x * config.y_width + node.y);
  if (map_index >= config.obstacle_map.size()) {
    return false;
  }

  return !config.obstacle_map[map_index];
}

std::pair<std::vector<double>, std::vector<double>> CalcFinalPath(
    const Node& goal_node, const std::unordered_map<int, Node>& closed_set,
    const Config& config) {
  std::vector<double> rx;
  std::vector<double> ry;
  rx.push_back(CalcGridPosition(goal_node.x, config.min_x, config.resolution));
  ry.push_back(CalcGridPosition(goal_node.y, config.min_y, config.resolution));

  int parent_index = goal_node.parent_index;
  while (parent_index != -1) {
    const auto iter = closed_set.find(parent_index);
    if (iter == closed_set.end()) {
      break;
    }
    const auto& node = iter->second;
    rx.push_back(CalcGridPosition(node.x, config.min_x, config.resolution));
    ry.push_back(CalcGridPosition(node.y, config.min_y, config.resolution));
    parent_index = node.parent_index;
  }

  std::reverse(rx.begin(), rx.end());
  std::reverse(ry.begin(), ry.end());
  return {rx, ry};
}

Grid BuildGridSnapshot(const Config& config, const Node& ref_node,
                       const std::unordered_map<int, Node>& open_set,
                       const std::unordered_map<int, Node>& closed_set,
                       const Node& start, const Node& goal,
                       bool use_goal_path = false) {
  const auto [rx, ry] =
      CalcFinalPath(use_goal_path ? goal : ref_node, closed_set, config);

  Grid grid;
  grid.width = config.x_width;
  grid.height = config.y_width;
  grid.data.assign(static_cast<size_t>(config.x_width * config.y_width),
                   CellType::kEmpty);

  for (int ix = 0; ix < config.x_width; ++ix) {
    for (int iy = 0; iy < config.y_width; ++iy) {
      const size_t index = static_cast<size_t>(ix * config.y_width + iy);
      if (config.obstacle_map[index]) {
        grid.data[index] = CellType::kObstacle;
      }
    }
  }

  for (const auto& [id, node] : open_set) {
    (void)id;
    const size_t index = static_cast<size_t>(node.x * config.y_width + node.y);
    if (index < grid.data.size()) {
      grid.data[index] = CellType::kOpen;
    }
  }

  for (const auto& [id, node] : closed_set) {
    (void)id;
    const size_t index = static_cast<size_t>(node.x * config.y_width + node.y);
    if (index < grid.data.size()) {
      grid.data[index] = CellType::kClosed;
    }
  }

  const size_t start_index =
      static_cast<size_t>(start.x * config.y_width + start.y);
  if (start_index < grid.data.size()) {
    grid.data[start_index] = CellType::kStart;
  }

  const size_t goal_index =
      static_cast<size_t>(goal.x * config.y_width + goal.y);
  if (goal_index < grid.data.size()) {
    grid.data[goal_index] = CellType::kGoal;
  }

  // Mark the current best path so it can be visualized on the grid.
  for (size_t i = 0; i < rx.size() && i < ry.size(); ++i) {
    const int ix = CalcXYIndex(rx[i], config.min_x, config.resolution);
    const int iy = CalcXYIndex(ry[i], config.min_y, config.resolution);
    const size_t path_index = static_cast<size_t>(ix * config.y_width + iy);
    if (path_index >= grid.data.size()) {
      continue;
    }
    if (path_index == start_index || path_index == goal_index) {
      continue;
    }
    grid.data[path_index] = CellType::kPath;
  }
  return grid;
}

std::vector<double> BuildObstacleX() {
  std::vector<double> ox;
  for (int i = -10; i < 60; ++i) {
    ox.push_back(static_cast<double>(i));
  }
  for (int i = -10; i < 60; ++i) {
    ox.push_back(60.0);
  }
  for (int i = -10; i < 61; ++i) {
    ox.push_back(static_cast<double>(i));
  }
  for (int i = -10; i < 61; ++i) {
    ox.push_back(-10.0);
  }
  for (int i = -10; i < 40; ++i) {
    ox.push_back(20.0);
  }
  for (int i = 0; i < 40; ++i) {
    ox.push_back(40.0);
  }
  return ox;
}

std::vector<double> BuildObstacleY() {
  std::vector<double> oy;
  for (int i = -10; i < 60; ++i) {
    oy.push_back(-10.0);
  }
  for (int i = -10; i < 60; ++i) {
    oy.push_back(static_cast<double>(i));
  }
  for (int i = -10; i < 61; ++i) {
    oy.push_back(60.0);
  }
  for (int i = -10; i < 61; ++i) {
    oy.push_back(static_cast<double>(i));
  }
  for (int i = -10; i < 40; ++i) {
    oy.push_back(static_cast<double>(i));
  }
  for (int i = 0; i < 40; ++i) {
    oy.push_back(60.0 - static_cast<double>(i));
  }
  return oy;
}

}  // namespace

std::vector<Grid> Run() {
  // Default parameters matching the Python Robotics example.
  constexpr double grid_size = 2.0;
  constexpr double robot_radius = 1.0;
  const double sx = 10.0;
  const double sy = 10.0;
  const double gx = 50.0;
  const double gy = 50.0;

  const auto ox = BuildObstacleX();
  const auto oy = BuildObstacleY();

  const auto config = BuildConfig(ox, oy, grid_size, robot_radius);
  const auto motion = GetMotionModel();

  Node start{CalcXYIndex(sx, config.min_x, config.resolution),
             CalcXYIndex(sy, config.min_y, config.resolution), 0.0, -1};
  Node goal{CalcXYIndex(gx, config.min_x, config.resolution),
            CalcXYIndex(gy, config.min_y, config.resolution), 0.0, -1};

  std::unordered_map<int, Node> open_set;
  std::unordered_map<int, Node> closed_set;
  open_set.emplace(CalcGridIndex(start, config), start);

  std::vector<Grid> snapshots;
  snapshots.push_back(
      BuildGridSnapshot(config, start, open_set, closed_set, start, goal));

  bool found_goal = false;
  while (!open_set.empty()) {
    const auto current_iter = std::min_element(
        open_set.begin(), open_set.end(),
        [&goal](const auto& lhs, const auto& rhs) {
          const double lhs_score =
              lhs.second.cost + CalcHeuristic(goal, lhs.second);
          const double rhs_score =
              rhs.second.cost + CalcHeuristic(goal, rhs.second);
          return lhs_score < rhs_score;
        });

    const int current_id = current_iter->first;
    Node current = current_iter->second;

    if (current.x == goal.x && current.y == goal.y) {
      goal.parent_index = current.parent_index;
      goal.cost = current.cost;
      found_goal = true;
      break;
    }

    open_set.erase(current_id);
    closed_set.emplace(current_id, current);

    for (const auto& m : motion) {
      Node node{current.x + m.dx, current.y + m.dy, current.cost + m.cost,
                current_id};
      const int node_id = CalcGridIndex(node, config);

      if (!VerifyNode(node, config)) {
        continue;
      }
      if (closed_set.find(node_id) != closed_set.end()) {
        continue;
      }

      const auto open_iter = open_set.find(node_id);
      if (open_iter == open_set.end() || open_iter->second.cost > node.cost) {
        open_set[node_id] = node;
      }
    }

    snapshots.push_back(
        BuildGridSnapshot(config, current, open_set, closed_set, start, goal));
  }

  Grid final_grid =
      BuildGridSnapshot(config, goal, open_set, closed_set, start, goal,
                        /*use_goal_path=*/found_goal);

  snapshots.push_back(final_grid);
  return snapshots;
}

}  // namespace a_star
}  // namespace grid_based_search
}  // namespace path_planning
