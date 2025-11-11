#include "rclcpp/rclcpp.hpp"
#include "foxglove_msgs/msg/grid.hpp"
#include "foxglove_msgs/msg/vector2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "foxglove_msgs/msg/packed_element_field.hpp"

enum class CellType {
  kEmpty = 0,
  kObstacle = 1,
  kStart = 2,
  kGoal = 3,
};

struct Grid {
  std::vector<CellType> data;
  int width;
  int height;
};

using Results = std::vector<Grid>;

using GridMsg = foxglove_msgs::msg::Grid;

Results GetResults() {
  auto results = Results{};
  for (int i = 0; i < 10; i++) {
    auto incremental_index = i % 4;
    if (incremental_index == 0) {
      results.push_back(Grid{std::vector<CellType>(100, CellType::kObstacle), 10, 10});
    } else if (incremental_index == 1) {
      results.push_back(Grid{std::vector<CellType>(100, CellType::kEmpty), 10, 10});
    } else if (incremental_index == 2) {
      results.push_back(Grid{std::vector<CellType>(100, CellType::kStart), 10, 10});
    } else {
      results.push_back(Grid{std::vector<CellType>(100, CellType::kGoal), 10, 10});
    }
  }
  return results;
}

template <typename Msg, typename F>
[[nodiscard]] Msg InitializeMsg(F&& f)
{
  Msg msg{};                
  std::forward<F>(f)(msg);
  return msg;
}

class AStarNode : public rclcpp::Node {
public:
  AStarNode() : rclcpp::Node("a_star_node") {
    publisher_ = this->create_publisher<GridMsg>("grid", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(0), [this]() { 
      auto results = GetResults();
      std::for_each(results.begin(), results.end(), [this](const auto& result) {
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        PublishGrid(result);
      });
    });
  }

private:
  void PublishGrid(const Grid& grid) {
    auto grid_center = InitializeMsg<geometry_msgs::msg::Pose>([&grid](geometry_msgs::msg::Pose& pose) {
      pose.position.x = - grid.width / 2.0;
      pose.position.y = - grid.height / 2.0;
      pose.position.z = 0.0;
    });
    auto field = InitializeMsg<foxglove_msgs::msg::PackedElementField>([&](foxglove_msgs::msg::PackedElementField& field) {
      field.name = "type";
      field.offset = 0;
      field.type = foxglove_msgs::msg::PackedElementField::UINT8;
    });
    auto cell_size = InitializeMsg<foxglove_msgs::msg::Vector2>([&](foxglove_msgs::msg::Vector2& vector2) {
      vector2.x = 1.0;
      vector2.y = 1.0;
    });

    const auto msg = InitializeMsg<GridMsg>([&](GridMsg& msg) {
      msg.timestamp = this->now();
      msg.frame_id = "map";
      msg.pose = grid_center;
      msg.column_count = grid.width;
      msg.cell_size = cell_size;
      msg.fields.push_back(field);
      msg.row_stride = grid.height;
      msg.cell_stride = 1;
      msg.data.resize(grid.data.size());
      std::transform(grid.data.begin(), grid.data.end(), msg.data.begin(), [](const auto& cell) {
        return static_cast<uint8_t>(cell);
      });
    });
    publisher_->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "Published grid");
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<GridMsg>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AStarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

