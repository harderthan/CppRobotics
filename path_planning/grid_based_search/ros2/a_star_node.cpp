
#include "a_star/a_star.hpp"
#include "foxglove_msgs/msg/grid.hpp"
#include "foxglove_msgs/msg/vector2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "grid_based_search.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

using Grid = path_planning::grid_based_search::a_star::Grid;
using GridBasedSearchNode =
    path_planning::grid_based_search::GridBasedSearch<Grid>;

foxglove_msgs::msg::Grid GridToMsgCallback(const Grid& grid) {
  geometry_msgs::msg::Pose grid_center;
  grid_center.position.x = -grid.width / 2.0;
  grid_center.position.y = -grid.height / 2.0;
  grid_center.position.z = 0.0;

  foxglove_msgs::msg::PackedElementField field;
  field.name = "type";
  field.offset = 0;
  field.type = foxglove_msgs::msg::PackedElementField::UINT8;

  foxglove_msgs::msg::Vector2 cell_size;
  // Adaptive cell size to always display as 10x10 grid
  cell_size.x = grid.width / 10.0;
  cell_size.y = grid.height / 10.0;

  foxglove_msgs::msg::Grid msg;
  msg.timestamp = rclcpp::Clock().now();
  msg.frame_id = "map";
  msg.pose = grid_center;
  msg.column_count = grid.width;
  msg.cell_size = cell_size;
  msg.fields.push_back(field);
  msg.row_stride = grid.height;
  msg.cell_stride = 1;
  msg.data.resize(grid.data.size());
  for (size_t i = 0; i < grid.data.size(); ++i) {
    msg.data[i] = static_cast<uint8_t>(grid.data[i]);
  }
  return msg;
}

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GridBasedSearchNode>(
      "a_star_node", path_planning::grid_based_search::a_star::Run,
      GridToMsgCallback);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}