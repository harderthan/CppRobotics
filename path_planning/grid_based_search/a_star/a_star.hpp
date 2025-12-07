#ifndef PATH_PLANNING_GRID_BASED_SEARCH_A_STAR_A_STAR_HPP_
#define PATH_PLANNING_GRID_BASED_SEARCH_A_STAR_A_STAR_HPP_

#include <vector>

namespace path_planning {
namespace grid_based_search {
namespace a_star {

enum class CellType {
  kEmpty = 0,
  kObstacle = 1,
  kStart = 2,
  kGoal = 3,
  kClosed = 4,
  kOpen = 5,
  kPath = 6,
};

struct Grid {
  std::vector<CellType> data;
  int width;
  int height;
};

using Results = std::vector<Grid>;

Results Run();

}  // namespace a_star
}  // namespace grid_based_search
}  // namespace path_planning

#endif  // PATH_PLANNING_GRID_BASED_SEARCH_A_STAR_A_STAR_HPP_
