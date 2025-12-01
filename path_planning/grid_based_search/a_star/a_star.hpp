#ifndef PATH_PLANNING_GRID_BASED_SEARCH_A_STAR_A_STAR_H
#define PATH_PLANNING_GRID_BASED_SEARCH_A_STAR_A_STAR_H

#include <vector>

namespace path_planning {
namespace grid_based_search {
namespace a_star {

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

std::vector<Grid> Run();

}  // namespace a_star
}  // namespace grid_based_search
}  // namespace path_planning

#endif  // PATH_PLANNING_GRID_BASED_SEARCH_A_STAR_A_STAR_H
