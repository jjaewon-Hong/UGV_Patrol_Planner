// Simulation loop for the web demo: the drive, battery and perception simulators
// (ugv_patrol_planner/nodes/*_sim.py) and the controller glue (cpp/node/controller_node.cpp)
// in one class, without ROS 2. One tick() is one drive period.
// It only depends on ugv_core, so it builds natively (unit tests) and as WebAssembly.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "ugv_core/mission.hpp"

namespace ugv {

struct SimConfig {
  TerrainParams terrain;
  EnergyParams energy;
  Cell base{0, 0};
  Cell start{0, 0};                  // scenario start_cell
  double capacity_wh = 500.0;
  double initial_battery_wh = 500.0;
  int sensor_radius = 3;             // cells
  double tick_s = 0.2;               // simulated time per tick (5 Hz drive rate)
};

struct SimAlarm {
  double t = 0.0;  // simulation time [s]
  std::string text;
};

struct PlanResult {
  std::optional<Path> path;
  double cost = 0.0;
  double plan_ms = 0.0;  // average over the repeats
};

class Simulation {
 public:
  // elevation: n*n values, row-major. obstacles: cells known from the start.
  Simulation(int n, std::vector<double> elevation, const std::vector<Cell>& obstacles, SimConfig cfg = {});
  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;

  // Obstacles the controller does not know; perception reports them within the sensor radius (FR-18).
  void set_hidden(std::vector<Cell> cells);

  // Operator command text: "add r c" | "remove P1" | "start" | "stop". Malformed text gives an alarm (NFR-04).
  void command(const std::string& text);
  // Obstacle report from outside the simulation (a click in the demo); same path as a perception report.
  void report_obstacles(const std::vector<Cell>& cells);
  void set_battery(double wh);

  // One drive period: move, drain the battery, then battery -> pose -> seen obstacles to the controller.
  // Returns true when the UGV moved.
  bool tick();

  double now() const { return ticks_ * cfg_.tick_s; }
  long ticks() const { return ticks_; }
  Cell pos() const { return pos_; }
  double battery_wh() const { return battery_; }
  double min_battery_wh() const { return min_battery_; }
  const Status& status() const { return status_; }
  const Path& path() const { return path_; }
  const std::vector<SimAlarm>& alarms() const { return alarms_; }
  const std::vector<MissionState>& state_history() const { return states_; }  // repeats collapsed
  const std::map<std::string, int>& visits() const { return visits_; }
  int total_visits() const;
  int replans() const { return replans_; }
  double last_replan_ms() const { return last_replan_ms_; }
  int moves() const { return moves_; }
  double max_slope_deg() const { return max_slope_; }
  int obstacle_entries() const { return obstacle_entries_; }  // moves that ended on an obstacle cell (must stay 0)
  const TerrainMap& terrain() const { return terrain_; }
  const ReturnEnergyMap& energy() const { return energy_; }
  const SimConfig& config() const { return cfg_; }

  // A* between two cells on the current map, apart from the mission (path explorer in the demo).
  PlanResult plan_query(Cell from, Cell to, int repeats = 1) const;

  // Everything the page draws, as JSON. Alarms and newly learned obstacles are listed once:
  // each call returns the ones since the previous call.
  std::string state_json();
  std::string plan_json(Cell from, Cell to, int repeats = 20) const;

 private:
  void handle(const Outputs& outputs);
  void alarm(const std::string& text) { alarms_.push_back({now(), text}); }
  void deliver_obstacles(const std::vector<Cell>& cells);

  SimConfig cfg_;
  TerrainMap terrain_;
  AStarPlanner planner_;
  PatrolScheduler sched_;
  ReturnEnergyMap energy_;
  MissionManager mission_;

  std::vector<Cell> hidden_;
  Cell pos_;
  std::optional<Cell> pending_;  // drive: next cell asked by the controller
  double battery_;
  double min_battery_;
  bool empty_reported_ = false;
  long ticks_ = 0;

  Status status_;
  Path path_;
  std::vector<SimAlarm> alarms_;
  std::vector<MissionState> states_;
  std::map<std::string, int> visits_;
  std::map<std::string, double> last_visit_;
  std::vector<Cell> learned_;  // obstacles added after construction
  size_t alarms_read_ = 0;
  size_t learned_read_ = 0;
  int replans_ = 0;
  double last_replan_ms_ = 0.0;
  int moves_ = 0;
  double max_slope_ = 0.0;
  int obstacle_entries_ = 0;
};

}  // namespace ugv
