// C interface of the web demo (WebAssembly). A thin shell around ugv::Simulation:
// plain C functions, arrays in, JSON text out. Used by web/js/wasm.js.
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "sim.hpp"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define UGV_API EMSCRIPTEN_KEEPALIVE
#else
#define UGV_API
#endif

namespace {
std::unique_ptr<ugv::Simulation> g_sim;
std::string g_text;           // last returned JSON, valid until the next call
std::string g_error;
std::vector<float> g_energy;  // last returned return-energy map

const char* keep(std::string s) {
  g_text = std::move(s);
  return g_text.c_str();
}
}  // namespace

extern "C" {

// Number of values ugv_create expects in `cfg`, in this order:
//  0 cell_size_m   1 max_slope_deg   2 slope_weight
//  3 wh_per_m      4 wh_per_climb_m  5 return_margin
//  6 capacity_wh   7 initial_battery_wh
//  8 sensor_radius 9 base_r  10 base_c  11 start_r  12 start_c  13 tick_s
UGV_API int ugv_config_size() { return 14; }

// Build the world. elev: n*n doubles, obstacles: n*n bytes (non-zero = obstacle), both row-major.
// Returns 0, or 1 with the reason in ugv_last_error().
UGV_API int ugv_create(int n, const double* elev, const std::uint8_t* obstacles, const double* cfg) {
  try {
    if (n <= 0 || n > 4096) throw std::invalid_argument("map size out of range");
    const size_t count = static_cast<size_t>(n) * n;
    ugv::SimConfig c;
    c.terrain.cell_size = cfg[0];
    c.terrain.max_slope_deg = cfg[1];
    c.terrain.slope_weight = cfg[2];
    c.energy.wh_per_m = cfg[3];
    c.energy.wh_per_climb_m = cfg[4];
    c.energy.return_margin = cfg[5];
    c.capacity_wh = cfg[6];
    c.initial_battery_wh = cfg[7];
    c.sensor_radius = static_cast<int>(cfg[8]);
    c.base = {static_cast<int>(cfg[9]), static_cast<int>(cfg[10])};
    c.start = {static_cast<int>(cfg[11]), static_cast<int>(cfg[12])};
    c.tick_s = cfg[13];
    if (!(c.terrain.cell_size > 0.0) || !(c.terrain.max_slope_deg > 0.0))
      throw std::invalid_argument("cell size and slope limit must be positive");
    std::vector<ugv::Cell> cells;
    for (size_t i = 0; i < count; ++i)
      if (obstacles[i]) cells.push_back({static_cast<int>(i) / n, static_cast<int>(i) % n});
    g_sim = std::make_unique<ugv::Simulation>(n, std::vector<double>(elev, elev + count), cells, c);
    g_error.clear();
    return 0;
  } catch (const std::exception& e) {
    g_sim.reset();
    g_error = e.what();
    return 1;
  }
}

UGV_API const char* ugv_last_error() { return g_error.c_str(); }

// cells: count pairs (r, c)
UGV_API void ugv_set_hidden(const int* cells, int count) {
  if (!g_sim) return;
  std::vector<ugv::Cell> v;
  for (int i = 0; i < count; ++i) v.push_back({cells[2 * i], cells[2 * i + 1]});
  g_sim->set_hidden(std::move(v));
}

UGV_API void ugv_command(const char* text) {
  if (g_sim) g_sim->command(text);
}

UGV_API void ugv_report_obstacle(int r, int c) {
  if (g_sim) g_sim->report_obstacles({{r, c}});
}

UGV_API void ugv_set_battery(double wh) {
  if (g_sim) g_sim->set_battery(wh);
}

// Advance `count` ticks; returns how many of them moved the UGV.
UGV_API int ugv_tick(int count) {
  int moved = 0;
  for (int i = 0; g_sim && i < count; ++i) moved += g_sim->tick();
  return moved;
}

UGV_API const char* ugv_state_json() { return keep(g_sim ? g_sim->state_json() : "{}"); }

UGV_API const char* ugv_plan_json(int r0, int c0, int r1, int c1) {
  return keep(g_sim ? g_sim->plan_json({r0, c0}, {r1, c1}) : "{}");
}

// Return energy [Wh] of every cell (n*n floats, row-major); -1 where the base cannot be reached.
UGV_API const float* ugv_return_energy() {
  if (!g_sim) return nullptr;
  const ugv::TerrainMap& t = g_sim->terrain();
  const int n = t.size();
  g_energy.resize(static_cast<size_t>(n) * n);
  for (int i = 0; i < n * n; ++i) {
    const double e = g_sim->energy().return_energy(t.cell(i));
    g_energy[i] = e == ugv::kInf ? -1.0f : static_cast<float>(e);
  }
  return g_energy.data();
}

// 1 when the move a -> b (neighbours) is allowed on the current map.
UGV_API int ugv_traversable(int r0, int c0, int r1, int c1) {
  return g_sim && g_sim->terrain().is_traversable({r0, c0}, {r1, c1});
}

}  // extern "C"
