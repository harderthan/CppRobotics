#include <cmath>
#include <iostream>
#include <vector>

#include "matplotlibcpp.h"
#include "pure_pursuit.h"

namespace {

namespace plt = matplotlibcpp;
namespace pp = path_tracking::pure_pursuit;

void Plot(const std::vector<std::pair<double, double>>& points) {
  for (const auto& point : points) {
    plt::plot({point.first}, {point.second}, "og");
  }
}

void Plot(const std::vector<pp::State>& states) {
  for (const auto& state : states) {
    plt::plot({state.x()}, {state.y()}, "xr");
  }
}

void Plot(double x, double y, std::string color) { plt::plot({x}, {y}, color); }

void Arrow(double x1, double y1, double yaw) {
  const double length = 1.5;
  plt::arrow(x1, y1, length * std::cos(yaw), length * std::sin(yaw), "k", "k");
}

}  // namespace

int main() {
  std::cout << "Hello, Pure-Pursuit Path Tracking!" << std::endl;

  bool show_animation = true;
  if (show_animation) {
    plt::xlim(0, 60);
    plt::ylim(0, 60);
    plt::grid(true);
    plt::axis("equal");
  }

  // Generate target course.
  std::vector<std::pair<double, double>> points;
  for (double x = 0.0; x < 50.0; x += 0.5) {
    points.emplace_back(x, std::sin(x / 5.0) * x / 2.0);
  }

  // Initialize variables.
  pp::State state(0.0, -3.0, 0.0, 0.0);
  pp::TargetCourse target_course(points);
  int pind = target_course.SearchTargetIndex(state);
  pp::PurePursuitController controller(points, pind);
  const double target_speed = 10.0 / 3.6;  // [m/s]
  // Simulation loop.
  const double T = 100.0;
  const double dt = 0.1;
  double time = 0.0;
  std::vector<pp::State> states;  // Store states for visualization.
  states.push_back(state);
  int last_index = points.size() - 1;
  while (T >= time && last_index >= pind) {
    double accel = controller.ProportionalControl(target_speed, state.v());
    double delta = controller.SteerControl(state);
    state.Update(accel, delta, dt);
    time += dt;
    states.push_back(state);

    if (show_animation) {
      Plot(points);
      Plot(state.rear_x(), state.rear_y(), "xr");
      Plot(points[controller.GetPind()].first,
           points[controller.GetPind()].second, "xb");
      Arrow(state.rear_x(), state.rear_y(), state.yaw());
      plt::title("Speed[km/h]:" + std::to_string(state.v() * 3.6));
      plt::pause(0.001);
      plt::cla();
    }
  }

  if (last_index < pind) {
    std::cout << "Cannot goal" << std::endl;
  }

  // Visualize the result.
  if (show_animation) {
    Plot(points);
    Plot(states);
    plt::pause(0.001);
    plt::show();
  }

  std::cout << "Simulation finished." << std::endl;
  return 0;
}
