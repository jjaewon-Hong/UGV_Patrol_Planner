#include "ugv_core/mission.hpp"

#include <algorithm>
#include <charconv>
#include <cstdio>
#include <sstream>

namespace ugv {

using S = MissionState;

const char* state_name(MissionState s) {
  switch (s) {
    case S::Idle: return "대기";
    case S::Patrolling: return "순찰 중";
    case S::Replanning: return "재계획 중";
    case S::Returning: return "복귀 중";
    case S::Stopped: return "정지";
  }
  return "?";
}

std::string format_cell(Cell c) { return "(" + std::to_string(c.r) + ", " + std::to_string(c.c) + ")"; }

namespace {
bool to_int(const std::string& s, int& out) {
  const char* end = s.data() + s.size();
  const char* b = s.data() + (!s.empty() && s[0] == '+');  // Python int() accepts a leading '+'
  auto res = std::from_chars(b, end, out);
  return res.ec == std::errc() && res.ptr == end && b != end;
}

std::string json_escape(const std::string& s) {
  std::string out;
  for (char ch : s) {
    switch (ch) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", ch);
          out += buf;
        } else {
          out += ch;  // UTF-8 passes through (like ensure_ascii=False)
        }
    }
  }
  return out;
}
}  // namespace

std::optional<Command> parse_command(const std::string& text, std::string* error) {
  std::istringstream in(text);
  std::vector<std::string> parts;
  for (std::string w; in >> w;) parts.push_back(w);
  if (parts.size() == 1 && parts[0] == "start") return Command{Command::Kind::Start, {}, {}};
  if (parts.size() == 1 && parts[0] == "stop") return Command{Command::Kind::Stop, {}, {}};
  if (parts.size() == 3 && parts[0] == "add") {
    int r, c;
    if (to_int(parts[1], r) && to_int(parts[2], c)) return Command{Command::Kind::Add, {r, c}, {}};
  }
  if (parts.size() == 2 && parts[0] == "remove") return Command{Command::Kind::Remove, {}, parts[1]};
  if (error) *error = "잘못된 명령: '" + text + "'";
  return std::nullopt;
}

std::string status_json(const Status& s) {
  std::string j = "{\"state\": \"" + std::string(state_name(s.state)) + "\", \"target\": ";
  j += s.target ? "\"" + json_escape(*s.target) + "\"" : "null";
  j += ", \"points\": [";
  for (size_t i = 0; i < s.points.size(); ++i) {
    const auto& p = s.points[i];
    char lv[64];
    if (p.last_visit) std::snprintf(lv, sizeof(lv), "%.9f", *p.last_visit);
    j += (i ? ", " : "") + std::string("{\"pid\": \"") + json_escape(p.pid) + "\", \"cell\": [" +
         std::to_string(p.cell.r) + ", " + std::to_string(p.cell.c) + "], \"last_visit\": " +
         (p.last_visit ? lv : "null") + "}";
  }
  j += "]";
  if (!s.note.empty()) j += ", \"note\": \"" + json_escape(s.note) + "\"";
  return j + "}";
}

bool MissionManager::running() const {
  return state_ == S::Patrolling || state_ == S::Replanning || state_ == S::Returning;
}

Outputs MissionManager::on_command(const Command& cmd, double now) {
  switch (cmd.kind) {
    case Command::Kind::Add:
      if (t_.in_bounds(cmd.cell) && !t_.is_obstacle(cmd.cell)) {
        sched_.add_point(cmd.cell);
        return {make_status()};
      }
      return {Alarm{"잘못된 순찰 지점: " + format_cell(cmd.cell)}};
    case Command::Kind::Remove:
      if (sched_.remove_point(cmd.pid)) return {make_status()};
      return {Alarm{"없는 지점 ID: " + cmd.pid}};
    case Command::Kind::Start:
      if (state_ != S::Idle && state_ != S::Stopped) return {};
      if (sched_.size() == 0 || !pos_ || !battery_wh_)
        return {Alarm{"시작 조건 미충족 (순찰 지점, 현재 위치, 배터리 잔량 필요)"}};
      return start_next_target(now);
    case Command::Kind::Stop:
      if (!running()) return {};
      path_.clear();
      idx_ = 0;
      return set_state(S::Stopped);
  }
  return {};
}

Outputs MissionManager::on_battery(double wh) {
  battery_wh_ = wh;
  return {};
}

Outputs MissionManager::on_pose(Cell cell, double now) {
  pos_ = cell;
  if ((state_ != S::Patrolling && state_ != S::Returning) || path_.empty()) return {};
  const auto it = std::find(path_.begin() + static_cast<long>(idx_), path_.end(), cell);
  if (it != path_.end()) idx_ = static_cast<size_t>(it - path_.begin());
  if (cell == path_.back()) {
    if (state_ == S::Returning) return set_state(S::Idle, "복귀 완료");
    sched_.mark_visited(target_->pid, now);
    return start_next_target(now);
  }
  return step();
}

Outputs MissionManager::on_obstacles(const std::vector<Cell>& cells, double now) {
  if (t_.add_obstacles(cells).empty()) return {};
  energy_.update();
  if ((state_ != S::Patrolling && state_ != S::Returning) || planner_.is_path_valid(path_, idx_)) return {};
  resume_state_ = state_;
  Outputs out = set_state(S::Replanning);
  auto append = [&out](Outputs more) { out.insert(out.end(), more.begin(), more.end()); };
  if (resume_state_ == S::Returning) {
    append(go_home());
    return out;
  }
  auto new_path = planner_.plan(*pos_, target_->cell);
  if (!new_path) {
    out.push_back(Alarm{"경로 없음: " + target_->pid});
    append(start_next_target(now, {target_->pid}));
    return out;
  }
  path_ = std::move(*new_path);
  idx_ = 0;
  append(set_state(S::Patrolling));
  out.push_back(PathUpdate{path_});
  append(step());
  return out;
}

Outputs MissionManager::start_next_target(double now, std::set<std::string> exclude) {
  Outputs out;
  for (;;) {
    const auto p = sched_.select_next(now, exclude);
    if (!p) {
      path_.clear();
      idx_ = 0;
      target_.reset();
      out.push_back(Alarm{"경로 없음"});
      for (auto& o : set_state(S::Idle)) out.push_back(std::move(o));
      return out;
    }
    auto new_path = planner_.plan(*pos_, p->cell);
    if (!new_path) {
      exclude.insert(p->pid);
      out.push_back(Alarm{"경로 없음: " + p->pid});
      continue;
    }
    target_ = *p;
    path_ = std::move(*new_path);
    idx_ = 0;
    // Report the new target and visit times even when the state stays "patrolling".
    Outputs status = set_state(S::Patrolling);
    if (status.empty()) status.push_back(make_status());
    for (auto& o : status) out.push_back(std::move(o));
    out.push_back(PathUpdate{path_});
    for (auto& o : step()) out.push_back(std::move(o));
    return out;
  }
}

Outputs MissionManager::step() {
  if (idx_ + 1 >= path_.size()) return {};
  const Cell nxt = path_[idx_ + 1];
  if (state_ == S::Patrolling && energy_.should_return(*pos_, battery_wh_.value_or(0.0), nxt)) return go_home();
  return {NextCell{nxt}};
}

Outputs MissionManager::go_home() {
  auto home = energy_.return_path(*pos_);
  if (!home) {
    path_.clear();
    idx_ = 0;
    Outputs out{Alarm{"기지 복귀 경로 없음"}};
    for (auto& o : set_state(S::Stopped)) out.push_back(std::move(o));
    return out;
  }
  target_.reset();
  if (home->size() == 1) {
    path_.clear();
    idx_ = 0;
    return set_state(S::Idle, "복귀 완료");
  }
  path_ = std::move(*home);
  idx_ = 0;
  Outputs out = set_state(S::Returning);
  out.push_back(PathUpdate{path_});
  out.push_back(NextCell{path_[1]});
  return out;
}

Outputs MissionManager::set_state(MissionState s, const std::string& note) {
  if (s == state_) return {};
  state_ = s;
  return {make_status(note)};
}

Status MissionManager::make_status(const std::string& note) const {
  Status s{state_, std::nullopt, sched_.points(), note};
  if (target_) s.target = target_->pid;
  return s;
}

}  // namespace ugv
