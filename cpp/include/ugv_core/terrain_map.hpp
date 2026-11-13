// U1 TerrainMap in C++17: elevation/obstacle grid, slope limit and move costs.
// Same rules as ugv_patrol_planner/terrain/terrain_map.py, so both give identical costs.
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace ugv {

struct Cell {
  int r = 0;
  int c = 0;
  bool operator==(const Cell& o) const { return r == o.r && c == o.c; }
  bool operator!=(const Cell& o) const { return !(*this == o); }
};

using Path = std::vector<Cell>;

constexpr double kInf = std::numeric_limits<double>::infinity();
// 4-connected moves in the same order as the Python version: up, down, left, right.
constexpr std::array<std::array<int, 2>, 4> kMoves{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};

struct TerrainParams {
  double cell_size = 30.0;     // metres per cell
  double max_slope_deg = 30.0;  // steeper moves are impassable (FR-03)
  double slope_weight = 2.0;    // cost = cell_size * (1 + w * slope / max_slope) (FR-04)
};

class TerrainMap {
 public:
  // elevation: n*n values, row-major.
  TerrainMap(int n, std::vector<double> elevation, TerrainParams params = {});

  int size() const { return n_; }
  double cell_size() const { return p_.cell_size; }
  bool in_bounds(Cell c) const { return c.r >= 0 && c.r < n_ && c.c >= 0 && c.c < n_; }
  int index(Cell c) const { return c.r * n_ + c.c; }
  Cell cell(int idx) const { return {idx / n_, idx % n_}; }

  double elevation(Cell c) const { return elev_[index(c)]; }
  bool is_obstacle(Cell c) const { return obstacle_[index(c)] != 0; }
  double slope_deg(Cell a, Cell b) const;

  // Cost of a single 4-connected move; kInf if off the map, an obstacle or too steep.
  double move_cost(Cell a, Cell b) const;
  bool is_traversable(Cell a, Cell b) const { return move_cost(a, b) != kInf; }

  // Cost of moving from cell index `idx` in direction d (0..3), kInf if not allowed.
  double cost(int d, int idx) const { return cost_[d][idx]; }

  // Register obstacles (out-of-range cells are ignored, NFR-04); returns the newly added ones.
  std::vector<Cell> add_obstacles(const std::vector<Cell>& cells);

  // Calls f(neighbour_index, cost) for every traversable move out of idx.
  template <class F>
  void for_each_neighbor(int idx, F&& f) const {
    const int r = idx / n_, c = idx % n_;
    for (int d = 0; d < 4; ++d) {
      const double k = cost_[d][idx];
      if (k != kInf) f((r + kMoves[d][0]) * n_ + (c + kMoves[d][1]), k);
    }
  }

  // Calls f(predecessor_index) for every cell v whose move v -> idx is traversable.
  template <class F>
  void for_each_predecessor(int idx, F&& f) const {
    const int r = idx / n_, c = idx % n_;
    for (int d = 0; d < 4; ++d) {
      const int vr = r - kMoves[d][0], vc = c - kMoves[d][1];
      if (vr < 0 || vr >= n_ || vc < 0 || vc >= n_) continue;
      const int v = vr * n_ + vc;
      if (cost_[d][v] != kInf) f(v);
    }
  }

 private:
  int n_;
  TerrainParams p_;
  std::vector<double> elev_;
  std::vector<std::uint8_t> obstacle_;
  std::array<std::vector<double>, 4> cost_;  // cost_[d][idx]: move from idx in direction d
};

}  // namespace ugv
