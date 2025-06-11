#include "pure_pursuit.h"

#include <cmath>
#include <limits>
#include <utility>
#include <iostream>
#include <vector>

namespace {
constexpr double kKp = 1.0;
constexpr double kLfc = 2.0;
}  // namespace

namespace path_tracking {
namespace pure_pursuit {

static double test_variable = 0.0;

State::State(double x, double y, double yaw, double v, double wheel_base)
    : x_(x), y_(y), yaw_(yaw), v_(v), wheel_base_(wheel_base) {
  SetRearPosition(x_, y_, yaw_);
}

void State::Update(double acceleration, double delta, double dt) {
  x_ += v_ * std::cos(yaw_) * dt;
  y_ += v_ * std::sin(yaw_) * dt;
  yaw_ += v_ / wheel_base_ * std::tan(delta) * dt;
  v_ += acceleration * dt;
  SetRearPosition(x_, y_, yaw_);
}

// Calculate the Euclidean distance between the rear position and a target point
double State::CalcDistance(double point_x, double point_y) const {
  double dx = rear_x_ - point_x;
  double dy = rear_y_ - point_y;
  return std::hypot(dx, dy);
}

// Set the rear position of the vehicle based on the current position and yaw
// angle.
void State::SetRearPosition(double x, double y, double yaw) {
  rear_x_ = x - (wheel_base_ / 2) * std::cos(yaw);
  rear_y_ = y - (wheel_base_ / 2) * std::sin(yaw);
}

TargetCourse::TargetCourse(const std::vector<std::pair<double, double>>& points)
    : points_(points) {
  old_nearest_point_index_ = points_.begin();
}

int TargetCourse::SearchTargetIndex(const State& state) {
  // To speed up nearest point search, doing it at only first time.
  if (old_nearest_point_index_ == points_.end()) {
    // search nearest point index
    double min_d = std::numeric_limits<double>::max();
    for (auto iter = points_.begin(); iter != points_.end(); ++iter) {
      double dx = state.rear_x() - iter->first;
      double dy = state.rear_y() - iter->second;
      double d = std::hypot(dx, dy);
      if (d < min_d) {
        min_d = d;
        old_nearest_point_index_ = iter;
      }
    }
  } else {
    // If `old_nearest_point_index_` is the last point of the course, return the
    // index of the last point.
    if (old_nearest_point_index_ + 1 == points_.end()) {
      return old_nearest_point_index_ - points_.begin();
    }

    double distance_this_index = state.CalcDistance(
        old_nearest_point_index_->first, old_nearest_point_index_->second);
    while (true) {
      double distance_next_index =
          state.CalcDistance((old_nearest_point_index_ + 1)->first,
                             (old_nearest_point_index_ + 1)->second);
      if (distance_this_index < distance_next_index) {
        break;
      }
      old_nearest_point_index_++;
      distance_this_index = distance_next_index;
    }
  }

  // Update the lookahead distance
  lookahead_distance_ = kKp * state.v() + kLfc;

  while (lookahead_distance_ >
         state.CalcDistance(old_nearest_point_index_->first,
                            old_nearest_point_index_->second)) {
    if (old_nearest_point_index_ + 1 == points_.end()) {
      break;
    }
    old_nearest_point_index_++;
  }

  return old_nearest_point_index_ - points_.begin();
}

PurePursuitController::PurePursuitController(
    const std::vector<std::pair<double, double>>& points, int pind = 0)
    : target_course_(points), pind_(pind) {}

double PurePursuitController::SteerControl(const State& state) {
  int ind = target_course_.SearchTargetIndex(state);

  if (pind_ >= ind) {
    ind = pind_;
  }

  double tx, ty;
  if (ind < target_course_.GetPoints().size()) {
    tx = target_course_.GetPoints()[ind].first;
    ty = target_course_.GetPoints()[ind].second;
  } else {
    tx = target_course_.GetPoints().back().first;
    ty = target_course_.GetPoints().back().second;
    ind = target_course_.GetPoints().size() - 1;
  }
  pind_ = ind;


  double alpha =
      std::atan2(ty - state.rear_y(), tx - state.rear_x()) - state.yaw();
  double delta = std::atan2(2.0 * state.wheel_base() * std::sin(alpha) /
                                target_course_.GetLookaheadDistance(),
                            1.0);

  return delta;
}

double PurePursuitController::ProportionalControl(double target,
                                                  double current) {
  return kKp * (target - current);
}

}  // namespace pure_pursuit
}  // namespace path_tracking
