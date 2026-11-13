// config/*.yaml for the C++ controller: same keys and defaults as ugv_patrol_planner/common.py.
#pragma once

#include <string>
#include <vector>

#include "ugv_core/energy.hpp"
#include "ugv_core/terrain_map.hpp"

namespace ugv {

struct Config {
  std::string dem_file = "data/dem_100x100.npy";
  std::string obstacle_file;  // optional N x N bool grid
  int map_size = 100;
  std::vector<Cell> static_obstacles;
  TerrainParams terrain;
  EnergyParams energy;
  Cell base_cell{0, 0};
  double replan_time_limit_s = 0.5;
};

Config load_config(const std::string& path);

// Elevation and obstacles from the config's files; `share` resolves relative paths.
TerrainMap load_terrain(const Config& cfg, const std::string& share);

}  // namespace ugv
