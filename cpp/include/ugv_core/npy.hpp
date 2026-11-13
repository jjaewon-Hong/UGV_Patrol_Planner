// Minimal reader for the 2-D .npy files in data/ (float64/float32 elevation, bool obstacles).
#pragma once

#include <string>
#include <vector>

namespace ugv {

struct NpyArray {
  int rows = 0;
  int cols = 0;
  std::vector<double> data;  // row-major, bools become 0.0 / 1.0
};

// Supports little-endian '<f8', '<f4', '|b1' and '|u1' in C order. Throws std::runtime_error otherwise.
NpyArray load_npy(const std::string& path);

}  // namespace ugv
