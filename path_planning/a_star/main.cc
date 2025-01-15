#include <iostream>
#include <vector>

#include "a_star.h"
#include "matplotlibcpp.h"

namespace {
namespace plt = matplotlibcpp;
namespace pa = path_planning::a_star;
}  // namespace

int main() {
  std::cout << "Hello, A* Path Planning!" << std::endl;

  pa::Position start = {10.0, 10.0};
  pa::Position goal = {50.0, 50.0};
  double grid_size = 2.0;
  double robot_radius = 1.0;

  std::vector<pa::Position> obstacle_positions = {
      {0.0, 0.0}, {20.0, 20.0}, {30.0, 30.0}, {40.0, 40.0}, {60.0, 60.0}};

  bool show_animation = true;

  if (show_animation) {
    plt::plot({start.x}, {start.y}, "og");
    plt::plot({goal.x}, {goal.y}, "xb");
    for (const auto &obstacle_position : obstacle_positions) {
      plt::plot({obstacle_position.x}, {obstacle_position.y}, "ok");
    }
  }

  if (show_animation) {
    plt::xlim(0, 60);
    plt::ylim(0, 60);
    plt::grid(true);
    plt::axis("equal");
  }

  plt::pause(10.0);
  
  pa::AStarPlanner planner(grid_size, robot_radius, show_animation);
  planner.SetObstaclePositions(obstacle_positions);
  auto path = planner.Plan(start, goal);

  if (show_animation) {
    plt::plot({start.x}, {start.y}, "og");
    plt::plot({goal.x}, {goal.y}, "xb");
    for (const auto &obstacle_position : obstacle_positions) {
      plt::plot({obstacle_position.x}, {obstacle_position.y}, "ok");
    }
    for (const auto &position : path) {
      plt::plot({position.x}, {position.y}, "xr");
    }
    plt::pause(0.001);
    plt::show();
  }

  return 0;
}
