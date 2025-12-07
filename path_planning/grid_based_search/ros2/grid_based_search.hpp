#ifndef PATH_PLANNING_GRID_BASED_SEARCH_ROS2_GRID_BASED_SEARCH_HPP_
#define PATH_PLANNING_GRID_BASED_SEARCH_ROS2_GRID_BASED_SEARCH_HPP_

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "foxglove_msgs/msg/grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace path_planning {
namespace grid_based_search {

template <typename Grid>
class GridBasedSearch : public rclcpp::Node {
 public:
  GridBasedSearch(
      std::string name, std::function<std::vector<Grid>()> results_callback,
      std::function<foxglove_msgs::msg::Grid(const Grid&)> grid_to_msg_callback)
      : Node(name) {
    publisher_ = create_publisher<foxglove_msgs::msg::Grid>("grid", 1);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(0),
        [this, results_callback, grid_to_msg_callback]() {
          const auto results = results_callback();
          std::for_each(results.begin(), results.end(),
                        [this, grid_to_msg_callback](const auto& grid) {
                          publisher_->publish(grid_to_msg_callback(grid));
                          rclcpp::sleep_for(std::chrono::milliseconds(100));
                        });
        });
  }

 private:
  rclcpp::Publisher<foxglove_msgs::msg::Grid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace grid_based_search
}  // namespace path_planning

#endif  // PATH_PLANNING_GRID_BASED_SEARCH_ROS2_GRID_BASED_SEARCH_HPP_
