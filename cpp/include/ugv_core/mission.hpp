// U5 MissionManager in C++17: mission state machine (FR-07~FR-16, NFR-03, NFR-04).
// Like the Python version it never touches ROS 2: every event returns a list of outputs
// that the node publishes. The state names, alarm texts and status JSON match the Python
// controller, so the Python simulators, visualizer and scenario tests work with either one.
#pragma once

#include <optional>
#include <set>
#include <string>
#include <variant>
#include <vector>

#include "ugv_core/astar.hpp"
#include "ugv_core/energy.hpp"
#include "ugv_core/scheduler.hpp"

namespace ugv {

enum class MissionState { Idle, Patrolling, Replanning, Returning, Stopped };
const char* state_name(MissionState s);  // Korean names used on /mission/status

struct NextCell {
  Cell cell;
};
struct PathUpdate {
  Path path;
};
struct Status {
  MissionState state;
  std::optional<std::string> target;
  std::vector<PatrolPoint> points;
  std::string note;
};
struct Alarm {
  std::string text;
};
using Output = std::variant<NextCell, PathUpdate, Status, Alarm>;
using Outputs = std::vector<Output>;

struct Command {
  enum class Kind { Start, Stop, Add, Remove } kind;
  Cell cell{};      // Add
  std::string pid;  // Remove
};

// "start" | "stop" | "add <row> <col>" | "remove <id>"; nullopt and an alarm text if malformed.
std::optional<Command> parse_command(const std::string& text, std::string* error = nullptr);

// Status as the JSON text published on /mission/status.
std::string status_json(const Status& s);
std::string format_cell(Cell c);  // "(r, c)" like Python tuples

class MissionManager {
 public:
  MissionManager(TerrainMap& terrain, AStarPlanner& planner, PatrolScheduler& scheduler, ReturnEnergyMap& energy)
      : t_(terrain), planner_(planner), sched_(scheduler), energy_(energy) {}

  Status status() const { return make_status(); }
  MissionState state() const { return state_; }
  const std::optional<PatrolPoint>& target() const { return target_; }
  std::optional<Cell> pos() const { return pos_; }

  Outputs on_command(const Command& cmd, double now);
  Outputs on_battery(double wh);
  Outputs on_pose(Cell cell, double now);
  Outputs on_obstacles(const std::vector<Cell>& cells, double now);

 private:
  Outputs start_next_target(double now, std::set<std::string> exclude = {});
  Outputs step();
  Outputs go_home();
  Outputs set_state(MissionState s, const std::string& note = "");
  Status make_status(const std::string& note = "") const;
  bool running() const;

  TerrainMap& t_;
  AStarPlanner& planner_;
  PatrolScheduler& sched_;
  ReturnEnergyMap& energy_;
  MissionState state_ = MissionState::Idle;
  MissionState resume_state_ = MissionState::Patrolling;
  std::optional<PatrolPoint> target_;
  Path path_;
  size_t idx_ = 0;
  std::optional<Cell> pos_;
  std::optional<double> battery_wh_;
};

}  // namespace ugv
