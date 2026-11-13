#include "config.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "ugv_core/npy.hpp"

namespace ugv {

namespace {
template <class T>
T get(const YAML::Node& n, const char* key, T fallback) {
  return n && n[key] ? n[key].as<T>() : fallback;
}

Cell to_cell(const YAML::Node& n) { return {n[0].as<int>(), n[1].as<int>()}; }

std::string resolve(const std::string& p, const std::string& share) {
  return !p.empty() && p[0] == '/' ? p : share + "/" + p;
}
}  // namespace

Config load_config(const std::string& path) {
  const YAML::Node raw = YAML::LoadFile(path);
  const YAML::Node m = raw["map"], t = raw["terrain"], e = raw["energy"], mi = raw["mission"];
  Config c;
  c.dem_file = get(m, "dem_file", c.dem_file);
  c.obstacle_file = m && m["obstacle_file"] && !m["obstacle_file"].IsNull() ? m["obstacle_file"].as<std::string>() : "";
  c.map_size = get(m, "size", c.map_size);
  c.terrain.cell_size = get(m, "cell_size_m", c.terrain.cell_size);
  if (m && m["static_obstacles"])
    for (const auto& o : m["static_obstacles"]) c.static_obstacles.push_back(to_cell(o));
  c.terrain.max_slope_deg = get(t, "max_slope_deg", c.terrain.max_slope_deg);
  c.terrain.slope_weight = get(t, "slope_weight", c.terrain.slope_weight);
  c.energy.wh_per_m = get(e, "wh_per_m", c.energy.wh_per_m);
  c.energy.wh_per_climb_m = get(e, "wh_per_climb_m", c.energy.wh_per_climb_m);
  c.energy.return_margin = get(e, "return_margin", c.energy.return_margin);
  if (mi && mi["base_cell"]) c.base_cell = to_cell(mi["base_cell"]);
  c.replan_time_limit_s = get(mi, "replan_time_limit_s", c.replan_time_limit_s);
  return c;
}

TerrainMap load_terrain(const Config& cfg, const std::string& share) {
  const NpyArray dem = load_npy(resolve(cfg.dem_file, share));
  if (dem.rows != cfg.map_size || dem.cols != cfg.map_size)
    throw std::runtime_error("expected " + std::to_string(cfg.map_size) + " x " + std::to_string(cfg.map_size) +
                             " map, got " + std::to_string(dem.rows) + " x " + std::to_string(dem.cols));
  TerrainMap terrain(dem.rows, dem.data, cfg.terrain);
  std::vector<Cell> obstacles = cfg.static_obstacles;
  if (!cfg.obstacle_file.empty()) {
    const NpyArray grid = load_npy(resolve(cfg.obstacle_file, share));
    if (grid.rows != dem.rows || grid.cols != dem.cols) throw std::runtime_error("obstacle grid does not match map");
    for (int i = 0; i < grid.rows * grid.cols; ++i)
      if (grid.data[i] != 0.0) obstacles.push_back(terrain.cell(i));
  }
  terrain.add_obstacles(obstacles);
  return terrain;
}

}  // namespace ugv
