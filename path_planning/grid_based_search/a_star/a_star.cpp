#include "a_star.hpp"

namespace path_planning {
namespace grid_based_search {
namespace a_star {

std::vector<Grid> GetSampleResults() {
  auto results = std::vector<Grid>();
  for (int i = 0; i < 10; i++) {
    auto incremental_index = i % 4;
    if (incremental_index == 0) {
      results.push_back(
          Grid{std::vector<CellType>(100, CellType::kObstacle), 10, 10});
    } else if (incremental_index == 1) {
      results.push_back(
          Grid{std::vector<CellType>(100, CellType::kEmpty), 10, 10});
    } else if (incremental_index == 2) {
      results.push_back(
          Grid{std::vector<CellType>(100, CellType::kStart), 10, 10});
    } else {
      results.push_back(
          Grid{std::vector<CellType>(100, CellType::kGoal), 10, 10});
    }
  }
  return results;
}

std::vector<Grid> Run() { return GetSampleResults(); }

}  // namespace a_star
}  // namespace grid_based_search
}  // namespace path_planning
