
#include "a_star/a_star.hpp"
#include "foxglove_msgs/msg/grid.hpp"
#include "foxglove_msgs/msg/vector2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "grid_based_search.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

using Grid = path_planning::grid_based_search::a_star::Grid;
using CellType = path_planning::grid_based_search::a_star::CellType;
using GridBasedSearchNode =
    path_planning::grid_based_search::GridBasedSearch<Grid>;

foxglove_msgs::msg::Grid GridToMsgCallback(const Grid& grid) {
  geometry_msgs::msg::Pose grid_center;
  grid_center.position.x = -grid.width / 2.0;
  grid_center.position.y = -grid.height / 2.0;
  grid_center.position.z = 0.0;

  foxglove_msgs::msg::PackedElementField r_field;
  r_field.name = "red";
  r_field.offset = 0;
  r_field.type = foxglove_msgs::msg::PackedElementField::UINT8;

  foxglove_msgs::msg::PackedElementField g_field;
  g_field.name = "green";
  g_field.offset = 1;
  g_field.type = foxglove_msgs::msg::PackedElementField::UINT8;

  foxglove_msgs::msg::PackedElementField b_field;
  b_field.name = "blue";
  b_field.offset = 2;
  b_field.type = foxglove_msgs::msg::PackedElementField::UINT8;

  foxglove_msgs::msg::PackedElementField a_field;
  a_field.name = "alpha";
  a_field.offset = 3;
  a_field.type = foxglove_msgs::msg::PackedElementField::UINT8;

  struct Rgba {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t alpha;
  };

  const auto ToColor = [](CellType cell) -> Rgba {
    switch (cell) {
      case CellType::kObstacle:
        return {0, 0, 0, 255};  // black
      case CellType::kGoal:
      case CellType::kPath:
        return {255, 0, 0, 255};  // red
      case CellType::kStart:
      case CellType::kOpen:
      case CellType::kClosed:
        return {0, 0, 255, 255};  // blue variants
      case CellType::kEmpty:
      default:
        return {255, 255, 255, 255};  // white
    }
  };

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
  msg.fields = {r_field, g_field, b_field, a_field};
  msg.cell_stride = 4;  // rgba bytes per cell
  msg.row_stride = msg.cell_stride * msg.column_count;

  const size_t cell_count =
      static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
  msg.data.resize(cell_count * msg.cell_stride);

  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const size_t src_index = static_cast<size_t>(x * grid.height + y);
      const size_t dst_index =
          static_cast<size_t>(y * grid.width + x) * msg.cell_stride;
      const auto color = ToColor(grid.data[src_index]);
      msg.data[dst_index + 0] = color.red;
      msg.data[dst_index + 1] = color.green;
      msg.data[dst_index + 2] = color.blue;
      msg.data[dst_index + 3] = color.alpha;
    }
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
