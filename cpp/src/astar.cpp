#include "ugv_core/astar.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <queue>
#include <tuple>
#include <vector>

namespace ugv {

namespace {
struct Entry {
  double f, h;
  std::uint64_t seq;
  int idx;
  // priority_queue is a max-heap; invert so the smallest (f, h, seq) comes first.
  bool operator<(const Entry& o) const { return std::tie(f, h, seq) > std::tie(o.f, o.h, o.seq); }
};
}  // namespace

std::optional<Path> AStarPlanner::plan(Cell start, Cell goal) {
  const auto t0 = std::chrono::steady_clock::now();
  struct Timer {
    std::chrono::steady_clock::time_point t0;
    double& out;
    ~Timer() {
      out = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }
  } timer{t0, last_plan_ms_};

  if (!t_.in_bounds(start) || !t_.in_bounds(goal) || t_.is_obstacle(goal)) return std::nullopt;
  const int n = t_.size();
  const double cs = t_.cell_size();
  // Manhattan distance * cell_size: every move costs at least cell_size, so h is admissible.
  auto heuristic = [&](int idx) { return (std::abs(idx / n - goal.r) + std::abs(idx % n - goal.c)) * cs; };

  std::vector<double> g(static_cast<size_t>(n) * n, kInf);
  std::vector<int> came_from(g.size(), -1);
  std::vector<std::uint8_t> closed(g.size(), 0);
  std::priority_queue<Entry> open;
  std::uint64_t seq = 0;
  const int s = t_.index(start), goal_idx = t_.index(goal);
  g[s] = 0.0;
  const double h0 = heuristic(s);
  open.push({h0, h0, seq++, s});

  while (!open.empty()) {
    const int cur = open.top().idx;
    open.pop();
    if (closed[cur]) continue;  // stale duplicate entry
    if (cur == goal_idx) {
      Path path;
      for (int v = goal_idx; v != -1; v = came_from[v]) path.push_back(t_.cell(v));
      std::reverse(path.begin(), path.end());
      return path;
    }
    closed[cur] = 1;
    t_.for_each_neighbor(cur, [&](int nb, double cost) {
      if (closed[nb]) return;
      const double ng = g[cur] + cost;
      if (ng < g[nb]) {
        g[nb] = ng;
        came_from[nb] = cur;
        const double h = heuristic(nb);
        open.push({ng + h, h, seq++, nb});
      }
    });
  }
  return std::nullopt;
}

bool AStarPlanner::is_path_valid(const Path& path, size_t from_idx) const {
  for (size_t i = from_idx; i + 1 < path.size(); ++i)
    if (!t_.is_traversable(path[i], path[i + 1])) return false;
  return true;
}

double AStarPlanner::path_cost(const Path& path) const {
  double total = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) total += t_.move_cost(path[i], path[i + 1]);
  return total;
}

}  // namespace ugv
