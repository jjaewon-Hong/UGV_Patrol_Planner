// U3 PatrolScheduler in C++17: patrol point table and longest-wait-first selection (FR-07, FR-09, FR-10).
#pragma once

#include <optional>
#include <set>
#include <string>
#include <vector>

#include "ugv_core/terrain_map.hpp"

namespace ugv {

struct PatrolPoint {
  std::string pid;
  Cell cell;
  std::optional<double> last_visit;  // empty = never visited
};

class PatrolScheduler {
 public:
  size_t size() const { return points_.size(); }

  // Register a point and return its ID ("P1", "P2", ...). The same cell returns the existing ID.
  std::string add_point(Cell cell);
  bool remove_point(const std::string& pid);
  const PatrolPoint* get(const std::string& pid) const;

  // Longest time since last visit; never-visited points first; ties go to the earlier point.
  std::optional<PatrolPoint> select_next(double now, const std::set<std::string>& exclude = {}) const;
  void mark_visited(const std::string& pid, double now);
  const std::vector<PatrolPoint>& points() const { return points_; }  // registration order

 private:
  std::vector<PatrolPoint> points_;
  int next_id_ = 1;
};

}  // namespace ugv
