#ifndef PATH_TRACKING_PURE_PURSUIT_PURE_PURSUIT_H_
#define PATH_TRACKING_PURE_PURSUIT_PURE_PURSUIT_H_

#include <utility>
#include <vector>

namespace path_tracking {
namespace pure_pursuit {

class State {
 public:
  State(double x = 0.0, double y = 0.0, double yaw = 0.0, double v = 0.0,
        double wheel_base = 2.9);

  void Update(double acceleration, double delta, double dt);
  double CalcDistance(double point_x, double point_y) const;

  // Getters
  double x() const { return x_; }
  double y() const { return y_; }
  double yaw() const { return yaw_; }
  double v() const { return v_; }
  double rear_x() const { return rear_x_; }
  double rear_y() const { return rear_y_; }
  double wheel_base() const { return wheel_base_; }

 private:
  void SetRearPosition(double x, double y, double yaw);

  double x_;                 // x position
  double y_;                 // y position
  double yaw_;               // yaw angle
  double v_;                 // velocity
  double rear_x_;            // rear x position
  double rear_y_;            // rear y position
  const double wheel_base_;  // [m] wheel base of vehicle.
};

using States = std::vector<State>;

class TargetCourse {
 public:
  TargetCourse(const TargetCourse& other) = delete;
  TargetCourse& operator=(const TargetCourse& other) = delete;
  TargetCourse(TargetCourse&& other) = delete;
  TargetCourse& operator=(TargetCourse&& other) = delete;

  explicit TargetCourse(const std::vector<std::pair<double, double>>& points);

  int SearchTargetIndex(const State& state);
  double GetLookaheadDistance() const { return lookahead_distance_; }
  std::vector<std::pair<double, double>> GetPoints() const { return points_; }

 private:
  std::vector<std::pair<double, double>> points_;
  std::vector<std::pair<double, double>>::iterator old_nearest_point_index_;
  double lookahead_distance_;
};

class PurePursuitController {
 public:
  PurePursuitController(const PurePursuitController& other) = delete;
  PurePursuitController& operator=(const PurePursuitController& other) = delete;
  PurePursuitController(PurePursuitController&& other) = delete;
  PurePursuitController& operator=(PurePursuitController&& other) = delete;

  explicit PurePursuitController(
      const std::vector<std::pair<double, double>>& points, int pind);

  int GetPind() const { return pind_; }
  double GetLookaheadDistance() const {
    return target_course_.GetLookaheadDistance();
  }
  double ProportionalControl(double target, double current);
  double SteerControl(const State& state);

 private:
  int pind_;
  TargetCourse target_course_;
};

double ProportionalControl(double target, double current);

}  // namespace pure_pursuit
}  // namespace path_tracking

#endif  // PATH_TRACKING_PURE_PURSUIT_PURE_PURSUIT_H_
