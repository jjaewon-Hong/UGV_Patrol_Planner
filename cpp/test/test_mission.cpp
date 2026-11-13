// U3 PatrolScheduler and U5 MissionManager tests, ported from test/test_u3_scheduler.py and
// test/test_u5_manager.py (same cases, same expected results).
#include <gtest/gtest.h>

#include <map>
#include <memory>

#include "ugv_core/mission.hpp"

using namespace ugv;
using S = MissionState;

// ------------------------------------------------------------------ U3
namespace {
PatrolScheduler make_sched(int n) {
  PatrolScheduler s;
  for (int i = 0; i < n; ++i) s.add_point({i, i});
  return s;
}
}  // namespace

TEST(Scheduler, AddAndRemove) {
  auto s = make_sched(3);
  EXPECT_TRUE(s.remove_point("P2"));
  EXPECT_FALSE(s.remove_point("P9"));
  ASSERT_EQ(s.size(), 2u);
  EXPECT_EQ(s.points()[0].pid, "P1");
  EXPECT_EQ(s.points()[1].pid, "P3");
  EXPECT_EQ(s.add_point({0, 0}), "P1");  // same cell keeps its ID
  EXPECT_EQ(s.size(), 2u);
}

TEST(Scheduler, LongestWaitFirstAndUnvisitedFirst) {
  auto s = make_sched(3);
  s.mark_visited("P1", 10);
  s.mark_visited("P2", 5);
  s.mark_visited("P3", 8);
  EXPECT_EQ(s.select_next(20)->pid, "P2");
  EXPECT_EQ(s.select_next(20, {"P2"})->pid, "P3");
  auto u = make_sched(3);
  u.mark_visited("P1", 1);
  EXPECT_EQ(u.select_next(100)->pid, "P2");
  EXPECT_FALSE(PatrolScheduler().select_next(0));
}

TEST(Scheduler, EvenCoverage) {
  auto s = make_sched(4);
  std::map<std::string, int> visits;
  for (int t = 0; t < 40; ++t) {
    const auto p = s.select_next(t);
    ++visits[p->pid];
    s.mark_visited(p->pid, t);
  }
  int lo = 1 << 30, hi = 0;
  for (auto& [pid, v] : visits) lo = std::min(lo, v), hi = std::max(hi, v);
  EXPECT_LE(hi - lo, 1);
}

// ------------------------------------------------------------------ U5
namespace {
struct World {
  TerrainMap terrain;
  AStarPlanner planner;
  PatrolScheduler sched;
  ReturnEnergyMap energy;
  MissionManager mm;
  explicit World(int n = 10)
      : terrain(n, std::vector<double>(static_cast<size_t>(n) * n, 0.0)),  // Config defaults: 30 m cells
        planner(terrain),
        energy(terrain, {0, 0}),
        mm(terrain, planner, sched, energy) {}
  void ready(std::vector<Cell> points, Cell pos = {0, 0}, double battery = 500.0) {
    for (Cell c : points) mm.on_command({Command::Kind::Add, c, {}}, 0);
    mm.on_pose(pos, 0);
    mm.on_battery(battery);
  }
  Outputs start(double now = 1) { return mm.on_command({Command::Kind::Start, {}, {}}, now); }
};

template <class T>
std::vector<T> of(const Outputs& outs) {
  std::vector<T> v;
  for (const auto& o : outs)
    if (auto p = std::get_if<T>(&o)) v.push_back(*p);
  return v;
}

std::vector<S> states(const Outputs& outs) {  // repeats collapsed
  std::vector<S> seq;
  for (const auto& s : of<Status>(outs))
    if (seq.empty() || seq.back() != s.state) seq.push_back(s.state);
  return seq;
}

void append(Outputs& a, const Outputs& b) { a.insert(a.end(), b.begin(), b.end()); }
}  // namespace

TEST(Mission, StartPlansToFirstPoint) {
  World w;
  w.ready({{0, 5}, {5, 5}});
  const auto outs = w.start();
  EXPECT_EQ(w.mm.state(), S::Patrolling);
  EXPECT_EQ(states(outs), std::vector<S>{S::Patrolling});
  EXPECT_EQ(of<PathUpdate>(outs)[0].path.back(), (Cell{0, 5}));
  EXPECT_EQ(of<NextCell>(outs)[0].cell, (Cell{0, 1}));
}

TEST(Mission, Stop) {
  World w;
  w.ready({{0, 5}});
  w.start();
  const auto outs = w.mm.on_command({Command::Kind::Stop, {}, {}}, 2);
  EXPECT_EQ(states(outs), std::vector<S>{S::Stopped});
  EXPECT_TRUE(w.mm.on_pose({0, 1}, 3).empty());
}

TEST(Mission, ArrivalUpdatesVisitAndReportsNewTarget) {
  World w;
  w.ready({{0, 3}, {3, 3}});
  w.start();
  w.mm.on_pose({0, 1}, 2);
  w.mm.on_pose({0, 2}, 3);
  const auto outs = w.mm.on_pose({0, 3}, 10);
  EXPECT_EQ(*w.sched.get("P1")->last_visit, 10.0);
  EXPECT_EQ(w.mm.target()->pid, "P2");
  EXPECT_EQ(of<PathUpdate>(outs)[0].path.back(), (Cell{3, 3}));
  const auto st = of<Status>(outs)[0];  // new target reported while still patrolling
  EXPECT_EQ(*st.target, "P2");
  EXPECT_EQ(*st.points[0].last_visit, 10.0);
}

TEST(Mission, ObstacleOnAndOffPath) {
  World w;
  w.ready({{0, 6}});
  w.start();
  w.mm.on_pose({0, 1}, 2);
  EXPECT_TRUE(w.mm.on_obstacles({{5, 5}}, 3).empty());
  const auto outs = w.mm.on_obstacles({{0, 3}}, 4);
  EXPECT_EQ(states(outs), (std::vector<S>{S::Replanning, S::Patrolling}));
  const auto p = of<PathUpdate>(outs)[0].path;
  EXPECT_EQ(std::find(p.begin(), p.end(), Cell{0, 3}), p.end());
  EXPECT_EQ(p.front(), (Cell{0, 1}));
  EXPECT_EQ(p.back(), (Cell{0, 6}));
}

TEST(Mission, UnreachableTargetIsSkipped) {
  World w;
  w.ready({{5, 5}, {8, 8}});
  w.terrain.add_obstacles({{4, 5}, {6, 5}, {5, 4}, {5, 6}});
  auto alarms = of<Alarm>(w.start());
  ASSERT_EQ(alarms.size(), 1u);
  EXPECT_EQ(alarms[0].text, "경로 없음: P1");
  EXPECT_EQ(w.mm.target()->pid, "P2");

  World w2;
  w2.ready({{5, 5}});
  w2.terrain.add_obstacles({{4, 5}, {6, 5}, {5, 4}, {5, 6}});
  alarms = of<Alarm>(w2.start());
  ASSERT_EQ(alarms.size(), 2u);
  EXPECT_EQ(alarms[1].text, "경로 없음");
  EXPECT_EQ(w2.mm.state(), S::Idle);
}

TEST(Mission, LowBatteryReturnsHome) {
  World w;
  w.ready({{9, 9}}, {3, 3});
  w.mm.on_battery(1.2 * w.energy.return_energy({3, 3}));
  const auto outs = w.start();
  EXPECT_EQ(states(outs), (std::vector<S>{S::Patrolling, S::Returning}));
  EXPECT_EQ(of<PathUpdate>(outs).back().path.back(), (Cell{0, 0}));
}

TEST(Mission, FullScenarioStatusSequence) {
  // Start -> obstacle on the path -> low battery -> back at base (FR-15, NFR-03).
  World w;
  double battery = 6.0;
  w.ready({{0, 9}, {9, 9}}, {0, 0}, battery);
  Outputs outs = w.start(), log = outs;
  for (int k = 0; k < 500; ++k) {
    const auto nxt = of<NextCell>(outs);
    if (nxt.empty()) break;
    const Cell prev = *w.mm.pos(), cell = nxt.back().cell;
    battery -= w.energy.energy_step(prev, cell);
    outs = w.mm.on_battery(battery);
    append(outs, w.mm.on_pose(cell, 1.0 + k));
    if (k == 1) append(outs, w.mm.on_obstacles({{0, 4}}, 100 + k));
    append(log, outs);
  }
  EXPECT_EQ(states(log), (std::vector<S>{S::Patrolling, S::Replanning, S::Patrolling, S::Returning, S::Idle}));
  EXPECT_EQ(*w.mm.pos(), (Cell{0, 0}));
  EXPECT_GT(battery, 0.0);
  EXPECT_EQ(of<Status>(log).back().note, "복귀 완료");
}

TEST(Mission, InvalidInputsGiveAlarms) {
  World w;
  EXPECT_FALSE(of<Alarm>(w.mm.on_command({Command::Kind::Add, {50, 50}, {}}, 0)).empty());
  EXPECT_FALSE(of<Alarm>(w.mm.on_command({Command::Kind::Remove, {}, "P9"}, 0)).empty());
  EXPECT_FALSE(of<Alarm>(w.start(0)).empty());  // no points
  EXPECT_EQ(w.mm.state(), S::Idle);
  const auto add = parse_command("add 3 4");
  ASSERT_TRUE(add);
  EXPECT_EQ(add->cell, (Cell{3, 4}));
  EXPECT_TRUE(parse_command(" start "));
  for (const char* bad : {"add 3 x", "go", "remove", "", "add 3 4.5"}) {
    std::string err;
    EXPECT_FALSE(parse_command(bad, &err)) << bad;
    EXPECT_FALSE(err.empty());
  }
}

TEST(Mission, StatusJsonMatchesPythonFormat) {
  World w;
  w.ready({{0, 3}});
  Status s = w.mm.status();
  s.note = "복귀 완료";
  EXPECT_EQ(status_json(s),
            "{\"state\": \"대기\", \"target\": null, \"points\": [{\"pid\": \"P1\", \"cell\": [0, 3], "
            "\"last_visit\": null}], \"note\": \"복귀 완료\"}");
}
