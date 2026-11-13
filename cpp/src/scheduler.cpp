#include "ugv_core/scheduler.hpp"

#include <algorithm>

namespace ugv {

std::string PatrolScheduler::add_point(Cell cell) {
  for (const auto& p : points_)
    if (p.cell == cell) return p.pid;
  points_.push_back({"P" + std::to_string(next_id_++), cell, std::nullopt});
  return points_.back().pid;
}

bool PatrolScheduler::remove_point(const std::string& pid) {
  const auto it = std::find_if(points_.begin(), points_.end(), [&](const PatrolPoint& p) { return p.pid == pid; });
  if (it == points_.end()) return false;
  points_.erase(it);
  return true;
}

const PatrolPoint* PatrolScheduler::get(const std::string& pid) const {
  for (const auto& p : points_)
    if (p.pid == pid) return &p;
  return nullptr;
}

std::optional<PatrolPoint> PatrolScheduler::select_next(double now, const std::set<std::string>& exclude) const {
  const PatrolPoint* best = nullptr;
  double best_wait = -kInf;
  for (const auto& p : points_) {
    if (exclude.count(p.pid)) continue;
    const double wait = p.last_visit ? now - *p.last_visit : kInf;
    if (wait > best_wait) {  // strict: ties keep the earlier registered point
      best = &p;
      best_wait = wait;
    }
  }
  if (!best) return std::nullopt;
  return *best;
}

void PatrolScheduler::mark_visited(const std::string& pid, double now) {
  for (auto& p : points_)
    if (p.pid == pid) p.last_visit = now;
}

}  // namespace ugv
