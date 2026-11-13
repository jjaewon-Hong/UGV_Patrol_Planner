// U2 AStarPlanner in C++17: minimum-cost path on a TerrainMap (FR-05, FR-06).
#pragma once

#include <optional>

#include "ugv_core/terrain_map.hpp"

namespace ugv {

class AStarPlanner {
 public:
  explicit AStarPlanner(const TerrainMap& terrain) : t_(terrain) {}

  // Minimum-cost path from start to goal, or std::nullopt if unreachable.
  // Ties are broken like the Python version (f, then h, then insertion order),
  // so both return the same path, not just the same cost.
  std::optional<Path> plan(Cell start, Cell goal);

  bool is_path_valid(const Path& path, size_t from_idx = 0) const;
  double path_cost(const Path& path) const;
  double last_plan_ms() const { return last_plan_ms_; }

 private:
  const TerrainMap& t_;
  double last_plan_ms_ = 0.0;
};

}  // namespace ugv
