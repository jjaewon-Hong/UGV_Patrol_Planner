// Web demo simulation loop (cpp/web/sim.cpp): the ROS 2 scenario tests S1~S5
// (scripts/scenario_test.py) run on the loop instead of on ROS 2, with the same pass criteria.
// S5 must give the same numbers as the ROS 2 run: 319 moves, steepest move 13.5 deg.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "sim.hpp"
#include "ugv_core/npy.hpp"

using namespace ugv;
using S = MissionState;

namespace {
const std::string kData = UGV_DATA_DIR;  // <repo>/data, set by CMake

// config/params.yaml + scenarios/default.yaml (or low_battery.yaml with battery = 30)
std::unique_ptr<Simulation> default_world(double battery = 500.0, bool hidden = true) {
  const NpyArray dem = load_npy(kData + "/dem_100x100.npy");
  SimConfig cfg;  // defaults are the params.yaml values: 30 m, 30 deg, base (0, 0)
  cfg.initial_battery_wh = battery;
  auto sim = std::make_unique<Simulation>(dem.rows, dem.data, std::vector<Cell>{{40, 41}, {40, 42}}, cfg);
  if (hidden) {
    std::vector<Cell> wall;
    for (int r = 0; r <= 15; ++r) wall.push_back({r, 30});
    sim->set_hidden(wall);
  }
  return sim;
}

// config/campus.yaml + scenarios/campus.yaml
std::unique_ptr<Simulation> campus_world(double max_slope_deg = 15.0) {
  const NpyArray dem = load_npy(kData + "/campus_pred/dem.npy");
  const NpyArray obs = load_npy(kData + "/campus_pred/obstacles.npy");
  std::vector<Cell> cells;
  for (int i = 0; i < obs.rows * obs.cols; ++i)
    if (obs.data[i] != 0.0) cells.push_back({i / obs.cols, i % obs.cols});
  SimConfig cfg;
  cfg.terrain.cell_size = 10.0;
  cfg.terrain.max_slope_deg = max_slope_deg;
  cfg.base = cfg.start = {42, 30};
  return std::make_unique<Simulation>(dem.rows, dem.data, cells, cfg);
}

void start_patrol(Simulation& sim) {
  for (const char* cmd : {"add 15 85", "add 85 85", "add 85 15", "start"}) sim.command(cmd);
}

template <class Done>
bool run_until(Simulation& sim, int max_ticks, Done done) {
  for (int i = 0; i < max_ticks; ++i) {
    sim.tick();
    if (done()) return true;
  }
  return false;
}

bool has_state(const Simulation& sim, S s) {
  const auto& h = sim.state_history();
  return std::find(h.begin(), h.end(), s) != h.end();
}

bool has_alarm(const Simulation& sim, const std::string& part) {
  for (const auto& a : sim.alarms())
    if (a.text.find(part) != std::string::npos) return true;
  return false;
}
}  // namespace

TEST(Sim, S1NormalPatrolVisitsEvenly) {
  auto sim = default_world();
  start_patrol(*sim);
  ASSERT_TRUE(run_until(*sim, 5000, [&] { return sim->total_visits() >= 6; })) << sim->state_json();
  const auto& v = sim->visits();
  ASSERT_EQ(v.size(), 3u);
  int lo = 1 << 30, hi = 0;
  for (const auto& [pid, n] : v) lo = std::min(lo, n), hi = std::max(hi, n);
  EXPECT_LE(hi - lo, 1);
}

TEST(Sim, S2HiddenObstaclesAreAvoided) {
  auto sim = default_world();
  start_patrol(*sim);
  ASSERT_TRUE(run_until(*sim, 5000, [&] { return sim->total_visits() >= 6; })) << sim->state_json();
  EXPECT_GT(sim->replans(), 0);
  EXPECT_TRUE(has_state(*sim, S::Replanning));
  EXPECT_LT(sim->last_replan_ms(), 500.0);  // NFR-01
  EXPECT_EQ(sim->obstacle_entries(), 0);
  for (int r = 0; r <= 15; ++r)  // the UGV passed the wall, so the controller learned (part of) it
    if (sim->terrain().is_obstacle({r, 30})) return;
  FAIL() << "no hidden obstacle was reported";
}

TEST(Sim, S3LowBatteryReturnsToBase) {
  auto sim = default_world(30.0, false);
  start_patrol(*sim);
  ASSERT_TRUE(run_until(*sim, 5000, [&] { return sim->status().note == "복귀 완료"; }));
  EXPECT_TRUE(has_state(*sim, S::Returning));
  EXPECT_EQ(sim->pos(), (Cell{0, 0}));
  EXPECT_GT(sim->min_battery_wh(), 0.0);  // NFR-03
  EXPECT_EQ(sim->status().state, S::Idle);
}

TEST(Sim, S4InvalidInputGivesAlarmsAndKeepsWorking) {
  auto sim = default_world();
  for (const char* cmd : {"add 999 999", "remove P9", "go", "start"}) sim->command(cmd);
  sim->report_obstacles({{500, 500}});
  for (int i = 0; i < 10; ++i) sim->tick();
  EXPECT_GE(sim->alarms().size(), 5u);
  EXPECT_TRUE(has_alarm(*sim, "잘못된 순찰 지점"));
  EXPECT_TRUE(has_alarm(*sim, "없는 지점 ID"));
  EXPECT_TRUE(has_alarm(*sim, "잘못된 명령"));
  EXPECT_TRUE(has_alarm(*sim, "시작 조건"));
  EXPECT_TRUE(has_alarm(*sim, "지도 밖 장애물"));
  sim->command("add 15 85");
  sim->command("start");
  EXPECT_EQ(sim->status().state, S::Patrolling);
  EXPECT_TRUE(sim->tick());  // and it drives
}

TEST(Sim, S5CampusPatrolMatchesTheRos2Run) {
  auto sim = campus_world();
  // scenarios/campus.yaml: random_flags {count 3, seed 7, min_spacing 20} gives these cells
  // (test/test_flags.py and test/test_export_web.py check that).
  for (const char* cmd : {"add 64 21", "add 3 1", "add 35 48", "start"}) sim->command(cmd);
  ASSERT_TRUE(run_until(*sim, 5000, [&] { return sim->visits().size() == 3; }));
  for (const auto& [pid, n] : sim->visits()) EXPECT_EQ(n, 1) << pid;
  EXPECT_EQ(sim->obstacle_entries(), 0);
  EXPECT_EQ(sim->moves(), 319);                    // same as scripts/scenario_test.py on ROS 2
  EXPECT_NEAR(sim->max_slope_deg(), 13.5, 0.05);   // printed there as 13.5 deg
  EXPECT_LE(sim->max_slope_deg(), 15.0 + 1e-9);    // FR-03
  EXPECT_TRUE(sim->alarms().empty());
}

TEST(Sim, SameInputsGiveTheSameRun) {
  auto a = default_world(), b = default_world();
  start_patrol(*a);
  start_patrol(*b);
  for (int i = 0; i < 1500; ++i) {
    a->tick();
    b->tick();
    ASSERT_EQ(a->pos(), b->pos()) << "tick " << i;
  }
  EXPECT_EQ(a->battery_wh(), b->battery_wh());
  EXPECT_EQ(a->visits(), b->visits());
  EXPECT_EQ(a->replans(), b->replans());
}

TEST(Sim, BatteryDrainsByTheEnergyModel) {
  auto sim = default_world(500.0, false);
  sim->command("add 0 5");
  sim->command("start");
  double expected = 500.0;
  Cell prev = sim->pos();
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(sim->tick());
    expected -= sim->energy().energy_step(prev, sim->pos());
    prev = sim->pos();
  }
  EXPECT_DOUBLE_EQ(sim->battery_wh(), expected);
  EXPECT_EQ(sim->moves(), 5);
  EXPECT_DOUBLE_EQ(sim->now(), 1.0);  // 5 ticks of 0.2 s
}

TEST(Sim, ReportedObstacleOnThePathIsReplannedAround) {
  auto sim = campus_world();
  sim->command("add 64 21");
  sim->command("start");
  for (int i = 0; i < 3; ++i) sim->tick();
  ASSERT_GE(sim->path().size(), 10u);
  const Path before = sim->path();
  const auto at = std::find(before.begin(), before.end(), sim->pos());
  ASSERT_NE(at, before.end());
  ASSERT_GT(before.end() - at, 4);
  const Cell block = *(at + 3);  // three cells ahead
  sim->report_obstacles({block});
  EXPECT_EQ(sim->replans(), 1);
  EXPECT_EQ(std::find(sim->path().begin(), sim->path().end(), block), sim->path().end());
  ASSERT_TRUE(run_until(*sim, 2000, [&] { return sim->visits().size() == 1; }));
  EXPECT_EQ(sim->obstacle_entries(), 0);
}

TEST(Sim, LoweringTheBatterySendsItHome) {
  auto sim = campus_world();
  for (const char* cmd : {"add 64 21", "add 3 1", "start"}) sim->command(cmd);
  for (int i = 0; i < 20; ++i) sim->tick();
  ASSERT_EQ(sim->status().state, S::Patrolling);
  sim->set_battery(sim->energy().return_energy(sim->pos()) * 1.3);  // just above the 1.2 margin
  ASSERT_TRUE(run_until(*sim, 2000, [&] { return sim->status().note == "복귀 완료"; }));
  EXPECT_TRUE(has_state(*sim, S::Returning));
  EXPECT_EQ(sim->pos(), (Cell{42, 30}));
  EXPECT_GT(sim->min_battery_wh(), 0.0);
}

TEST(Sim, EmptyBatteryStopsTheDrive) {
  auto sim = default_world(500.0, false);
  sim->command("add 0 9");
  sim->command("start");
  for (int i = 0; i < 3; ++i) sim->tick();
  const Cell where = sim->pos();
  sim->set_battery(0.0);
  for (int i = 0; i < 10; ++i) EXPECT_FALSE(sim->tick());
  EXPECT_EQ(sim->pos(), where);
  EXPECT_TRUE(has_alarm(*sim, "배터리 방전"));
}

TEST(Sim, SteeperLimitChangesWhatIsPassable) {
  auto strict = campus_world(15.0), loose = campus_world(30.0);
  int blocked_strict = 0, blocked_loose = 0;
  const int n = strict->terrain().size();
  for (int r = 0; r < n; ++r)
    for (int c = 0; c + 1 < n; ++c) {
      const Cell a{r, c}, b{r, c + 1};
      if (strict->terrain().is_obstacle(a) || strict->terrain().is_obstacle(b)) continue;
      blocked_strict += !strict->terrain().is_traversable(a, b);
      blocked_loose += !loose->terrain().is_traversable(a, b);
    }
  EXPECT_GT(blocked_strict, 0);
  EXPECT_EQ(blocked_loose, 0);  // the campus data never exceeds 23.8 deg
}

TEST(Sim, PlanQueryAndJson) {
  auto sim = campus_world();
  const PlanResult res = sim->plan_query({42, 30}, {64, 21}, 3);
  ASSERT_TRUE(res.path);
  EXPECT_EQ(res.path->front(), (Cell{42, 30}));
  EXPECT_EQ(res.path->back(), (Cell{64, 21}));
  EXPECT_GT(res.cost, 0.0);
  EXPECT_FALSE(sim->plan_query({42, 30}, {-1, 0}).path);
  EXPECT_NE(sim->plan_json({42, 30}, {64, 21}).find("\"found\":true"), std::string::npos);
  EXPECT_NE(sim->plan_json({42, 30}, {999, 0}).find("\"found\":false"), std::string::npos);

  sim->command("go \"x\"");  // quotes must be escaped in the JSON
  const std::string first = sim->state_json();
  EXPECT_NE(first.find("\\\"x\\\""), std::string::npos);
  EXPECT_NE(first.find("\"pos\":[42,30]"), std::string::npos);
  EXPECT_NE(first.find("\"state\": \"대기\""), std::string::npos);
  EXPECT_NE(sim->state_json().find("\"alarms\":[]"), std::string::npos);  // alarms are listed once
}

TEST(Sim, RejectsBadWorlds) {
  SimConfig cfg;
  cfg.start = {50, 50};
  EXPECT_THROW(Simulation(10, std::vector<double>(100, 0.0), {}, cfg), std::invalid_argument);
  cfg.start = {0, 0};
  cfg.base = {-1, 0};
  EXPECT_THROW(Simulation(10, std::vector<double>(100, 0.0), {}, cfg), std::invalid_argument);
  EXPECT_THROW(Simulation(10, std::vector<double>(99, 0.0), {}), std::invalid_argument);
}
