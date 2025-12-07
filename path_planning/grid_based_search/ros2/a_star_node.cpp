
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

constexpr int kPreviewCellsPerAxis = 10;
constexpr int kColorChannelCount = 4;

foxglove_msgs::msg::PackedElementField MakeField(const char* name,
                                                 unsigned int offset) {
  foxglove_msgs::msg::PackedElementField field;
  field.name = name;
  field.offset = offset;
  field.type = foxglove_msgs::msg::PackedElementField::UINT8;
  return field;
}

foxglove_msgs::msg::Grid GridToMsgCallback(const Grid& grid) {
  foxglove_msgs::msg::Grid msg;

  if (grid.width <= 0 || grid.height <= 0) {
    return msg;
  }

  const size_t width = static_cast<size_t>(grid.width);
  const size_t height = static_cast<size_t>(grid.height);
  const size_t cell_count = width * height;
  if (grid.data.size() != cell_count) {
    return msg;
  }

  geometry_msgs::msg::Pose grid_center;
  grid_center.position.x = -static_cast<double>(grid.width) / 2.0;
  grid_center.position.y = -static_cast<double>(grid.height) / 2.0;
  grid_center.position.z = 0.0;

  using ColorByte = unsigned char;
  struct Rgba {
    ColorByte red;
    ColorByte green;
    ColorByte blue;
    ColorByte alpha;
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
  cell_size.x = static_cast<double>(grid.width) /
                static_cast<double>(kPreviewCellsPerAxis);
  cell_size.y = static_cast<double>(grid.height) /
                static_cast<double>(kPreviewCellsPerAxis);

  msg.timestamp = rclcpp::Clock().now();
  msg.frame_id = "map";
  msg.pose = grid_center;
  msg.column_count = grid.width;
  msg.cell_size = cell_size;
  msg.fields = {MakeField("red", 0U), MakeField("green", 1U),
                MakeField("blue", 2U), MakeField("alpha", 3U)};
  msg.cell_stride = kColorChannelCount;  // rgba bytes per cell
  msg.row_stride = msg.cell_stride * msg.column_count;

  msg.data.resize(cell_count * static_cast<size_t>(msg.cell_stride));

  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const size_t src_index =
          static_cast<size_t>(x) * height + static_cast<size_t>(y);
      if (src_index >= grid.data.size()) {
        continue;
      }
      const size_t dst_index =
          (static_cast<size_t>(y) * width + static_cast<size_t>(x)) *
          static_cast<size_t>(msg.cell_stride);
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
