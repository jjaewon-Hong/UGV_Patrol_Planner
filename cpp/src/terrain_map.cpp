#include "ugv_core/terrain_map.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace ugv {

namespace {
constexpr double kEps = 1e-9;  // exactly max_slope_deg counts as passable despite rounding
constexpr double kPi = 3.14159265358979323846;
double degrees(double rad) { return rad * 180.0 / kPi; }
}  // namespace

TerrainMap::TerrainMap(int n, std::vector<double> elevation, TerrainParams params)
    : n_(n), p_(params), elev_(std::move(elevation)), obstacle_(static_cast<size_t>(n) * n, 0) {
  if (n <= 0 || elev_.size() != static_cast<size_t>(n) * n)
    throw std::invalid_argument("elevation must have n*n values, got " + std::to_string(elev_.size()));
  for (double e : elev_)
    if (!std::isfinite(e)) throw std::invalid_argument("elevation contains NaN or inf");

  for (int d = 0; d < 4; ++d) {
    auto& cost = cost_[d];
    cost.assign(elev_.size(), kInf);
    for (int r = 0; r < n_; ++r) {
      for (int c = 0; c < n_; ++c) {
        const Cell b{r + kMoves[d][0], c + kMoves[d][1]};
        if (!in_bounds(b)) continue;
        const double slope = slope_deg({r, c}, b);
        if (slope <= p_.max_slope_deg + kEps)
          cost[index({r, c})] = p_.cell_size * (1.0 + p_.slope_weight * slope / p_.max_slope_deg);
      }
    }
  }
}

double TerrainMap::slope_deg(Cell a, Cell b) const {
  return degrees(std::atan(std::abs(elevation(b) - elevation(a)) / p_.cell_size));
}

double TerrainMap::move_cost(Cell a, Cell b) const {
  if (!in_bounds(a) || !in_bounds(b)) return kInf;
  for (int d = 0; d < 4; ++d)
    if (b.r - a.r == kMoves[d][0] && b.c - a.c == kMoves[d][1]) return cost_[d][index(a)];
  return kInf;  // not a 4-connected move
}

std::vector<Cell> TerrainMap::add_obstacles(const std::vector<Cell>& cells) {
  std::vector<Cell> added;
  for (const Cell& c : cells) {
    if (!in_bounds(c) || is_obstacle(c)) continue;
    obstacle_[index(c)] = 1;
    added.push_back(c);
    for (int d = 0; d < 4; ++d) {  // block every move into c
      const Cell a{c.r - kMoves[d][0], c.c - kMoves[d][1]};
      if (in_bounds(a)) cost_[d][index(a)] = kInf;
    }
  }
  return added;
}

}  // namespace ugv
