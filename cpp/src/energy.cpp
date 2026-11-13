#include "ugv_core/energy.hpp"

#include <algorithm>
#include <functional>
#include <queue>
#include <stdexcept>
#include <utility>

namespace ugv {

ReturnEnergyMap::ReturnEnergyMap(const TerrainMap& terrain, Cell base, EnergyParams params)
    : t_(terrain), base_(base), p_(params) {
  if (!t_.in_bounds(base_)) throw std::invalid_argument("base cell is off the map");
  update();
}

double ReturnEnergyMap::energy_step(Cell a, Cell b) const {
  const double climb = std::max(0.0, t_.elevation(b) - t_.elevation(a));
  return p_.wh_per_m * t_.cell_size() + p_.wh_per_climb_m * climb;
}

void ReturnEnergyMap::update() {
  const int n = t_.size();
  ret_.assign(static_cast<size_t>(n) * n, kInf);
  const double flat = p_.wh_per_m * t_.cell_size();
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> heap;
  const int b = t_.index(base_);
  ret_[b] = 0.0;
  heap.push({0.0, b});
  while (!heap.empty()) {
    const auto [d, u] = heap.top();
    heap.pop();
    if (d > ret_[u]) continue;
    const double hu = t_.elevation(t_.cell(u));
    t_.for_each_predecessor(u, [&](int v) {  // cells that can move into u
      const double nd = d + flat + p_.wh_per_climb_m * std::max(0.0, hu - t_.elevation(t_.cell(v)));
      if (nd < ret_[v]) {
        ret_[v] = nd;
        heap.push({nd, v});
      }
    });
  }
}

bool ReturnEnergyMap::should_return(Cell pos, double battery_wh, std::optional<Cell> next) const {
  if (battery_wh <= p_.return_margin * return_energy(pos)) return true;
  if (next) {  // look one move ahead so the margin still holds after moving (NFR-03)
    const double predicted = battery_wh - energy_step(pos, *next);
    if (predicted <= p_.return_margin * return_energy(*next)) return true;
  }
  return false;
}

}  // namespace ugv
