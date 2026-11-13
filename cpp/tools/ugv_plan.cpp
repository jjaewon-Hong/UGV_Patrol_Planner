// Plan paths on a map from data/ and print them, for timing and for comparing with the Python planner.
//
//   ugv_plan <dem.npy> <obstacles.npy|-> <cell_size_m>  < queries
//
// Each input line "r0 c0 r1 c1" prints "cost plan_ms r0,c0;r1,c1;..." (or "none plan_ms").
#include <cstdio>
#include <iostream>
#include <string>

#include "ugv_core/astar.hpp"
#include "ugv_core/npy.hpp"

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: ugv_plan <dem.npy> <obstacles.npy|-> <cell_size_m> < queries\n";
    return 2;
  }
  try {
    const ugv::NpyArray dem = ugv::load_npy(argv[1]);
    if (dem.rows != dem.cols) throw std::runtime_error("map must be N x N");
    ugv::TerrainParams p;
    p.cell_size = std::stod(argv[3]);
    ugv::TerrainMap terrain(dem.rows, dem.data, p);
    if (std::string(argv[2]) != "-") {
      const ugv::NpyArray obs = ugv::load_npy(argv[2]);
      if (obs.rows != dem.rows || obs.cols != dem.cols) throw std::runtime_error("obstacle grid size differs");
      std::vector<ugv::Cell> cells;
      for (int i = 0; i < obs.rows * obs.cols; ++i)
        if (obs.data[i] != 0.0) cells.push_back(terrain.cell(i));
      terrain.add_obstacles(cells);
    }
    ugv::AStarPlanner planner(terrain);
    int r0, c0, r1, c1;
    while (std::cin >> r0 >> c0 >> r1 >> c1) {
      const auto path = planner.plan({r0, c0}, {r1, c1});
      if (!path) {
        std::printf("none %.4f\n", planner.last_plan_ms());
        continue;
      }
      std::printf("%.9f %.4f ", planner.path_cost(*path), planner.last_plan_ms());
      for (size_t i = 0; i < path->size(); ++i)
        std::printf("%s%d,%d", i ? ";" : "", (*path)[i].r, (*path)[i].c);
      std::printf("\n");
    }
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
