#include "sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

namespace ugv {

namespace {
std::string escape(const std::string& s) {
  std::string out;
  for (char ch : s) {
    if (ch == '"' || ch == '\\') {
      out += '\\';
      out += ch;
    } else if (static_cast<unsigned char>(ch) < 0x20) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "\\u%04x", ch);
      out += buf;
    } else {
      out += ch;  // UTF-8 passes through
    }
  }
  return out;
}

std::string num(double v, const char* fmt = "%.6g") {
  if (!std::isfinite(v)) return "null";
  char buf[48];
  std::snprintf(buf, sizeof(buf), fmt, v);
  return buf;
}

std::string cell_json(Cell c) { return "[" + std::to_string(c.r) + "," + std::to_string(c.c) + "]"; }

std::string cells_json(const std::vector<Cell>& cells, size_t from = 0) {
  std::string j = "[";
  for (size_t i = from; i < cells.size(); ++i) j += (i > from ? "," : "") + cell_json(cells[i]);
  return j + "]";
}

int manhattan(Cell a, Cell b) { return std::abs(a.r - b.r) + std::abs(a.c - b.c); }
}  // namespace

Simulation::Simulation(int n, std::vector<double> elevation, const std::vector<Cell>& obstacles, SimConfig cfg)
    : cfg_(cfg),
      terrain_(n, std::move(elevation), cfg.terrain),
      planner_(terrain_),
      energy_(terrain_, cfg.base, cfg.energy),
      mission_(terrain_, planner_, sched_, energy_),
      pos_(cfg.start),
      battery_(std::clamp(cfg.initial_battery_wh, 0.0, cfg.capacity_wh)),
      min_battery_(battery_),
      status_(mission_.status()) {
  if (!terrain_.in_bounds(cfg_.start)) throw std::invalid_argument("start cell is off the map");
  if (cfg_.tick_s <= 0.0) throw std::invalid_argument("tick_s must be positive");
  terrain_.add_obstacles(obstacles);
  energy_.update();
  states_.push_back(status_.state);
  // The simulators publish battery and pose before any command arrives; do the same here
  // so that "start" works before the first tick.
  handle(mission_.on_battery(battery_));
  handle(mission_.on_pose(pos_, now()));
}

void Simulation::set_hidden(std::vector<Cell> cells) { hidden_ = std::move(cells); }

void Simulation::command(const std::string& text) {
  std::string error;
  const auto cmd = parse_command(text, &error);
  if (!cmd) {  // malformed text: alarm, keep running (NFR-04)
    alarm(error);
    return;
  }
  handle(mission_.on_command(*cmd, now()));
}

void Simulation::report_obstacles(const std::vector<Cell>& cells) {
  std::string outside;
  for (const Cell& c : cells)
    if (!terrain_.in_bounds(c)) outside += (outside.empty() ? "" : ", ") + format_cell(c);
  if (!outside.empty()) alarm("지도 밖 장애물 무시: [" + outside + "]");
  deliver_obstacles(cells);
}

void Simulation::deliver_obstacles(const std::vector<Cell>& cells) {
  for (const Cell& c : cells)
    if (terrain_.in_bounds(c) && !terrain_.is_obstacle(c) &&
        std::find(learned_.begin(), learned_.end(), c) == learned_.end())
      learned_.push_back(c);
  const Outputs outs = mission_.on_obstacles(cells, now());
  for (const auto& o : outs) {
    const auto* s = std::get_if<Status>(&o);
    if (s && s->state == MissionState::Replanning) {  // NFR-01 monitoring, like the node
      ++replans_;
      last_replan_ms_ = planner_.last_plan_ms();
      break;
    }
  }
  handle(outs);
  // The drive never enters a cell that is now known to be an obstacle.
  if (pending_ && terrain_.is_obstacle(*pending_)) pending_.reset();
}

void Simulation::set_battery(double wh) {
  if (!std::isfinite(wh)) return;
  battery_ = std::clamp(wh, 0.0, cfg_.capacity_wh);
  min_battery_ = std::min(min_battery_, battery_);
  if (battery_ > 0.0) empty_reported_ = false;
}

bool Simulation::tick() {
  ++ticks_;
  bool moved = false;

  // 1. drive (drive_sim.py): one cell toward the commanded neighbour. An empty battery cannot move.
  if (pending_ && battery_ > 0.0) {
    const Cell prev = pos_;
    pos_ = *pending_;
    moved = true;
    ++moves_;
    max_slope_ = std::max(max_slope_, terrain_.slope_deg(prev, pos_));
    if (terrain_.is_obstacle(pos_) || std::find(hidden_.begin(), hidden_.end(), pos_) != hidden_.end())
      ++obstacle_entries_;
    // 2. battery (battery_sim.py): distance + climbing, the same model as U4.
    battery_ = std::max(0.0, battery_ - energy_.energy_step(prev, pos_));
    min_battery_ = std::min(min_battery_, battery_);
  }
  pending_.reset();
  if (battery_ == 0.0 && pos_ != cfg_.base && !empty_reported_) {
    alarm("배터리 방전: " + format_cell(pos_) + " 에서 정지");  // NFR-03 violation if it ever happens by itself
    empty_reported_ = true;
  }

  // 3. battery level, then position to the controller
  handle(mission_.on_battery(battery_));
  handle(mission_.on_pose(pos_, now()));

  // 4. perception (perception_sim.py): hidden obstacles within the sensor radius of the new position.
  // It reacts to the pose, so the controller hears about them after the pose, as on ROS 2.
  std::vector<Cell> seen;
  const int r2 = cfg_.sensor_radius * cfg_.sensor_radius;
  for (const Cell& h : hidden_) {
    const int dr = h.r - pos_.r, dc = h.c - pos_.c;
    if (dr * dr + dc * dc <= r2) seen.push_back(h);
  }
  if (!seen.empty()) deliver_obstacles(seen);
  return moved;
}

void Simulation::handle(const Outputs& outputs) {
  for (const auto& o : outputs) {
    if (const auto* n = std::get_if<NextCell>(&o)) {
      if (manhattan(n->cell, pos_) == 1) pending_ = n->cell;  // the drive ignores non-adjacent cells
    } else if (const auto* p = std::get_if<PathUpdate>(&o)) {
      path_ = p->path;
    } else if (const auto* s = std::get_if<Status>(&o)) {
      status_ = *s;
      if (states_.empty() || states_.back() != s->state) states_.push_back(s->state);
      if (s->state == MissionState::Idle || s->state == MissionState::Stopped) path_.clear();
      for (const auto& pt : s->points) {
        if (!pt.last_visit) continue;
        const auto it = last_visit_.find(pt.pid);
        if (it == last_visit_.end() || it->second != *pt.last_visit) {
          ++visits_[pt.pid];
          last_visit_[pt.pid] = *pt.last_visit;
        }
      }
    } else if (const auto* a = std::get_if<Alarm>(&o)) {
      alarm(a->text);
    }
  }
}

int Simulation::total_visits() const {
  int total = 0;
  for (const auto& [pid, v] : visits_) total += v;
  return total;
}

PlanResult Simulation::plan_query(Cell from, Cell to, int repeats) const {
  PlanResult res;
  AStarPlanner planner(terrain_);
  repeats = std::max(1, repeats);
  double total_ms = 0.0;
  for (int i = 0; i < repeats; ++i) {
    res.path = planner.plan(from, to);
    total_ms += planner.last_plan_ms();
  }
  res.plan_ms = total_ms / repeats;
  if (res.path) res.cost = planner.path_cost(*res.path);
  return res;
}

std::string Simulation::state_json() {
  std::string j = "{\"t\":" + num(now(), "%.3f") + ",\"tick\":" + std::to_string(ticks_);
  j += ",\"pos\":" + cell_json(pos_);
  j += ",\"battery\":" + num(battery_, "%.4f") + ",\"capacity\":" + num(cfg_.capacity_wh);
  j += ",\"min_battery\":" + num(min_battery_, "%.4f");
  j += ",\"return_wh\":" + num(energy_.return_energy(pos_), "%.4f");  // null when the base is unreachable
  j += ",\"status\":" + status_json(status_);
  j += ",\"visits\":{";
  bool first = true;
  for (const auto& [pid, v] : visits_) {
    j += (first ? "\"" : ",\"") + escape(pid) + "\":" + std::to_string(v);
    first = false;
  }
  j += "},\"path\":" + cells_json(path_);
  j += ",\"replans\":" + std::to_string(replans_) + ",\"last_replan_ms\":" + num(last_replan_ms_, "%.4f");
  j += ",\"moves\":" + std::to_string(moves_) + ",\"max_slope\":" + num(max_slope_, "%.4f");
  j += ",\"obstacle_entries\":" + std::to_string(obstacle_entries_);
  j += ",\"alarms\":[";
  for (size_t i = alarms_read_; i < alarms_.size(); ++i)
    j += std::string(i > alarms_read_ ? "," : "") + "{\"t\":" + num(alarms_[i].t, "%.3f") + ",\"text\":\"" +
         escape(alarms_[i].text) + "\"}";
  alarms_read_ = alarms_.size();
  j += "],\"new_obstacles\":" + cells_json(learned_, learned_read_);
  learned_read_ = learned_.size();
  return j + "}";
}

std::string Simulation::plan_json(Cell from, Cell to, int repeats) const {
  const PlanResult res = plan_query(from, to, repeats);
  std::string j = "{\"found\":" + std::string(res.path ? "true" : "false");
  j += ",\"plan_ms\":" + num(res.plan_ms, "%.4f");
  if (res.path) {
    double climb = 0.0, steepest = 0.0;
    for (size_t i = 0; i + 1 < res.path->size(); ++i) {
      const Cell a = (*res.path)[i], b = (*res.path)[i + 1];
      climb += std::max(0.0, terrain_.elevation(b) - terrain_.elevation(a));
      steepest = std::max(steepest, terrain_.slope_deg(a, b));
    }
    j += ",\"cost\":" + num(res.cost, "%.9f") + ",\"cells\":" + std::to_string(res.path->size() - 1);
    j += ",\"climb_m\":" + num(climb, "%.2f") + ",\"max_slope\":" + num(steepest, "%.4f");
    j += ",\"path\":" + cells_json(*res.path);
  }
  return j + "}";
}

}  // namespace ugv
