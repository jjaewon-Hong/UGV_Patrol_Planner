// Unit tests for the C++ core, mirroring test/test_u1_terrain.py, test_u2_astar.py and test_u4_energy.py.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <queue>
#include <random>

#include "ugv_core/astar.hpp"
#include "ugv_core/energy.hpp"
#include "ugv_core/npy.hpp"

using ugv::AStarPlanner;
using ugv::Cell;
using ugv::kInf;
using ugv::TerrainMap;

namespace {
TerrainMap flat(int n, double cs = 10.0) {
  ugv::TerrainParams p;
  p.cell_size = cs;
  return TerrainMap(n, std::vector<double>(static_cast<size_t>(n) * n, 0.0), p);
}

// Plain Dijkstra, used as the reference for A* optimality.
double dijkstra(const TerrainMap& t, Cell s, Cell g) {
  std::vector<double> d(static_cast<size_t>(t.size()) * t.size(), kInf);
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
  d[t.index(s)] = 0.0;
  q.push({0.0, t.index(s)});
  while (!q.empty()) {
    auto [du, u] = q.top();
    q.pop();
    if (du > d[u]) continue;
    t.for_each_neighbor(u, [&](int v, double c) {
      if (du + c < d[v]) {
        d[v] = du + c;
        q.push({d[v], v});
      }
    });
  }
  return d[t.index(g)];
}
}  // namespace

TEST(TerrainMap, SlopeCostAndLimit) {
  ugv::TerrainParams p;
  p.cell_size = 10.0;
  // 3x3, middle column 10 m higher than the left: atan(10/10) = 45 deg > 30 deg.
  TerrainMap t(3, {0, 10, 0, 0, 5, 0, 0, 0, 0}, p);
  EXPECT_EQ(t.move_cost({0, 0}, {0, 1}), kInf);                 // 45 deg: impassable
  const double s = std::atan(5.0 / 10.0) * 180.0 / M_PI;           // 26.57 deg: passable
  EXPECT_NEAR(t.move_cost({1, 0}, {1, 1}), 10.0 * (1 + 2.0 * s / 30.0), 1e-9);
  EXPECT_DOUBLE_EQ(t.move_cost({2, 0}, {2, 1}), 10.0);             // flat
  EXPECT_EQ(t.move_cost({0, 0}, {1, 1}), kInf);                    // diagonal is not a move
  EXPECT_EQ(t.move_cost({0, 0}, {-1, 0}), kInf);                   // off the map
}

TEST(TerrainMap, ExactlyMaxSlopeIsPassable) {
  ugv::TerrainParams p;
  p.cell_size = 10.0;
  const double h = 10.0 * std::tan(30.0 * M_PI / 180.0);
  TerrainMap t(2, {0, h, 0, 0}, p);
  EXPECT_NE(t.move_cost({0, 0}, {0, 1}), kInf);
}

TEST(TerrainMap, ObstaclesBlockIncomingMovesAndIgnoreOutOfRange) {
  TerrainMap t = flat(5);
  const auto added = t.add_obstacles({{2, 2}, {2, 2}, {99, 99}, {-1, 0}});
  ASSERT_EQ(added.size(), 1u);
  EXPECT_TRUE(t.is_obstacle({2, 2}));
  EXPECT_EQ(t.move_cost({1, 2}, {2, 2}), kInf);
  EXPECT_EQ(t.move_cost({2, 1}, {2, 2}), kInf);
  EXPECT_NE(t.move_cost({2, 2}, {2, 3}), kInf);  // leaving an obstacle cell is still priced
}

TEST(TerrainMap, RejectsBadInput) {
  EXPECT_THROW(TerrainMap(3, std::vector<double>(8, 0.0)), std::invalid_argument);
  EXPECT_THROW(TerrainMap(2, {0, 0, NAN, 0}), std::invalid_argument);
}

TEST(AStar, StraightLineOnFlatMap) {
  TerrainMap t = flat(10);
  AStarPlanner a(t);
  const auto path = a.plan({0, 0}, {0, 9});
  ASSERT_TRUE(path);
  EXPECT_EQ(path->size(), 10u);
  EXPECT_DOUBLE_EQ(a.path_cost(*path), 90.0);
  EXPECT_TRUE(a.is_path_valid(*path));
}

TEST(AStar, DetoursAroundWallAndFailsWhenClosed) {
  TerrainMap t = flat(10);
  std::vector<Cell> wall;
  for (int r = 0; r < 9; ++r) wall.push_back({r, 5});
  t.add_obstacles(wall);
  AStarPlanner a(t);
  const auto path = a.plan({0, 0}, {0, 9});
  ASSERT_TRUE(path);
  for (const Cell& c : *path) EXPECT_FALSE(t.is_obstacle(c));
  EXPECT_DOUBLE_EQ(a.path_cost(*path), (9 + 2 * 9) * 10.0);  // down to row 9, across, back up
  t.add_obstacles({{9, 5}});
  EXPECT_FALSE(a.plan({0, 0}, {0, 9}));
  EXPECT_FALSE(a.plan({0, 0}, {3, 5}));   // goal on an obstacle
  EXPECT_FALSE(a.plan({0, 0}, {10, 0}));  // goal off the map
}

TEST(AStar, PathBecomesInvalidWhenObstacleAppears) {
  TerrainMap t = flat(6);
  AStarPlanner a(t);
  auto path = a.plan({0, 0}, {0, 5});
  ASSERT_TRUE(path);
  t.add_obstacles({{0, 3}});
  EXPECT_FALSE(a.is_path_valid(*path));
  EXPECT_TRUE(a.is_path_valid(*path, 3));  // only the part after the obstacle is checked
}

TEST(AStar, OptimalOnRandomTerrain) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> height(0.0, 12.0);
  std::uniform_int_distribution<int> pos(0, 29);
  for (int trial = 0; trial < 20; ++trial) {
    std::vector<double> e(30 * 30);
    for (double& v : e) v = height(rng);
    ugv::TerrainParams p;
    p.cell_size = 10.0;
    TerrainMap t(30, e, p);
    std::vector<Cell> obs;
    for (int i = 0; i < 120; ++i) obs.push_back({pos(rng), pos(rng)});
    t.add_obstacles(obs);
    AStarPlanner a(t);
    const Cell s{pos(rng), pos(rng)}, g{pos(rng), pos(rng)};
    if (t.is_obstacle(s) || t.is_obstacle(g)) continue;
    const double ref = dijkstra(t, s, g);
    const auto path = a.plan(s, g);
    if (ref == kInf) {
      EXPECT_FALSE(path);
    } else {
      ASSERT_TRUE(path);
      EXPECT_NEAR(a.path_cost(*path), ref, 1e-9);
    }
  }
}

TEST(Energy, UphillCostsMoreThanDownhill) {
  ugv::TerrainParams p;
  p.cell_size = 10.0;
  TerrainMap t(1 + 2, {0, 2, 4, 0, 2, 4, 0, 2, 4}, p);  // rises 2 m per column
  ugv::EnergyParams e;  // 0.01 Wh/m, 0.5 Wh per climbed metre
  ugv::ReturnEnergyMap home_low(t, {0, 0}, e);
  ugv::ReturnEnergyMap home_high(t, {0, 2}, e);
  EXPECT_NEAR(home_low.return_energy({0, 2}), 2 * 0.1, 1e-12);              // downhill: distance only
  EXPECT_NEAR(home_high.return_energy({0, 0}), 2 * 0.1 + 0.5 * 4, 1e-12);   // uphill: plus climbing
}

TEST(Energy, ShouldReturnWithMarginAndLookAhead) {
  TerrainMap t = flat(10);
  ugv::ReturnEnergyMap m(t, {0, 0});
  const double r = m.return_energy({0, 5});  // 5 moves * 0.1 Wh
  EXPECT_NEAR(r, 0.5, 1e-12);
  EXPECT_TRUE(m.should_return({0, 5}, 1.2 * r));
  EXPECT_FALSE(m.should_return({0, 5}, 1.2 * r + 0.01));
  // Next move away from base would break the margin: 0.61 - 0.1 = 0.51 <= 1.2 * 0.6
  EXPECT_TRUE(m.should_return({0, 5}, 0.61, Cell{0, 6}));
  t.add_obstacles({{0, 1}, {1, 0}});  // base sealed off
  m.update();
  EXPECT_EQ(m.return_energy({0, 5}), kInf);
}

TEST(Npy, ReadsFloatAndBoolArrays) {
  const std::string path = ::testing::TempDir() + "t.npy";
  {
    std::ofstream f(path, std::ios::binary);
    std::string header = "{'descr': '<f8', 'fortran_order': False, 'shape': (2, 3), }";
    while ((10 + header.size() + 1) % 64) header += ' ';
    header += '\n';
    f.write("\x93NUMPY\x01\x00", 8);
    const unsigned short len = static_cast<unsigned short>(header.size());
    f.write(reinterpret_cast<const char*>(&len), 2);
    f << header;
    const double v[6] = {1, 2, 3, 4, 5, 6.5};
    f.write(reinterpret_cast<const char*>(v), sizeof(v));
  }
  const ugv::NpyArray a = ugv::load_npy(path);
  EXPECT_EQ(a.rows, 2);
  EXPECT_EQ(a.cols, 3);
  EXPECT_DOUBLE_EQ(a.data[5], 6.5);
  std::remove(path.c_str());
  EXPECT_THROW(ugv::load_npy("/nonexistent.npy"), std::runtime_error);
}
