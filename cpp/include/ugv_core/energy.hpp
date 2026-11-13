// U4 return-to-base energy map in C++17 (FR-13, FR-14, NFR-03).
#pragma once

#include <optional>
#include <vector>

#include "ugv_core/terrain_map.hpp"

namespace ugv {

struct EnergyParams {
  double wh_per_m = 0.01;       // energy per metre moved
  double wh_per_climb_m = 0.5;  // extra energy per metre climbed
  double return_margin = 1.2;   // keep 20 % extra for returning
};

class ReturnEnergyMap {
 public:
  ReturnEnergyMap(const TerrainMap& terrain, Cell base, EnergyParams params = {});

  // Energy [Wh] of one move: distance part + climbing part.
  double energy_step(Cell a, Cell b) const;

  // Reverse Dijkstra from the base: R[c] = minimum energy from c back to base.
  // Call again after obstacles change.
  void update();

  double return_energy(Cell c) const { return ret_[t_.index(c)]; }
  bool should_return(Cell pos, double battery_wh, std::optional<Cell> next = std::nullopt) const;

 private:
  const TerrainMap& t_;
  Cell base_;
  EnergyParams p_;
  std::vector<double> ret_;
};

}  // namespace ugv
