#include "a_star/a_star.hpp"

#include <gtest/gtest.h>

namespace path_planning {
namespace grid_based_search {
namespace a_star {
// Test CellType enum.
TEST(AStarTest, CellTypeEnum) {
  EXPECT_EQ(static_cast<int>(CellType::kEmpty), 0);
  EXPECT_EQ(static_cast<int>(CellType::kObstacle), 1);
  EXPECT_EQ(static_cast<int>(CellType::kStart), 2);
  EXPECT_EQ(static_cast<int>(CellType::kGoal), 3);
  EXPECT_EQ(static_cast<int>(CellType::kClosed), 4);
  EXPECT_EQ(static_cast<int>(CellType::kOpen), 5);
  EXPECT_EQ(static_cast<int>(CellType::kPath), 6);
}

// Test Grid structure initialization.
TEST(AStarTest, GridInitialization) {
  Grid grid;
  grid.width = 10;
  grid.height = 10;
  grid.data = std::vector<CellType>(100, CellType::kEmpty);

  EXPECT_EQ(grid.width, 10);
  EXPECT_EQ(grid.height, 10);
  EXPECT_EQ(grid.data.size(), 100);
}

// Test Grid with different cell types.
TEST(AStarTest, GridWithDifferentCellTypes) {
  Grid grid;
  grid.width = 5;
  grid.height = 5;
  grid.data = std::vector<CellType>(25, CellType::kEmpty);

  // Set start position.
  grid.data[0] = CellType::kStart;
  // Set goal position.
  grid.data[24] = CellType::kGoal;
  // Set obstacle.
  grid.data[12] = CellType::kObstacle;
  // Mark open and closed cells.
  grid.data[6] = CellType::kOpen;
  grid.data[18] = CellType::kClosed;
  // Mark a path cell.
  grid.data[7] = CellType::kPath;

  EXPECT_EQ(grid.data[0], CellType::kStart);
  EXPECT_EQ(grid.data[24], CellType::kGoal);
  EXPECT_EQ(grid.data[12], CellType::kObstacle);
  EXPECT_EQ(grid.data[6], CellType::kOpen);
  EXPECT_EQ(grid.data[18], CellType::kClosed);
  EXPECT_EQ(grid.data[7], CellType::kPath);
  EXPECT_EQ(grid.data[1], CellType::kEmpty);
}
}  // namespace a_star
}  // namespace grid_based_search
}  // namespace path_planning

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

