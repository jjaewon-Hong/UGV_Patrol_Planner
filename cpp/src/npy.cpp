#include "ugv_core/npy.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <regex>
#include <stdexcept>

namespace ugv {

NpyArray load_npy(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  char magic[6];
  f.read(magic, 6);
  if (std::memcmp(magic, "\x93NUMPY", 6) != 0) throw std::runtime_error(path + ": not a .npy file");
  unsigned char ver[2];
  f.read(reinterpret_cast<char*>(ver), 2);
  std::uint32_t header_len = 0;
  if (ver[0] == 1) {
    unsigned char b[2];
    f.read(reinterpret_cast<char*>(b), 2);
    header_len = b[0] | (b[1] << 8);
  } else {
    unsigned char b[4];
    f.read(reinterpret_cast<char*>(b), 4);
    header_len = b[0] | (b[1] << 8) | (b[2] << 16) | (static_cast<std::uint32_t>(b[3]) << 24);
  }
  std::string header(header_len, ' ');
  f.read(&header[0], header_len);

  std::smatch m;
  if (!std::regex_search(header, m, std::regex("'descr':\\s*'([^']+)'")))
    throw std::runtime_error(path + ": no dtype in header");
  const std::string descr = m[1];
  if (header.find("'fortran_order': False") == std::string::npos)
    throw std::runtime_error(path + ": only C order is supported");
  if (!std::regex_search(header, m, std::regex("'shape':\\s*\\((\\d+),\\s*(\\d+)\\)")))
    throw std::runtime_error(path + ": only 2-D arrays are supported");

  NpyArray a;
  a.rows = std::stoi(m[1]);
  a.cols = std::stoi(m[2]);
  const size_t count = static_cast<size_t>(a.rows) * a.cols;
  a.data.resize(count);
  if (descr == "<f8") {
    f.read(reinterpret_cast<char*>(a.data.data()), static_cast<std::streamsize>(count * 8));
  } else if (descr == "<f4") {
    std::vector<float> tmp(count);
    f.read(reinterpret_cast<char*>(tmp.data()), static_cast<std::streamsize>(count * 4));
    for (size_t i = 0; i < count; ++i) a.data[i] = tmp[i];
  } else if (descr == "|b1" || descr == "|u1") {
    std::vector<std::uint8_t> tmp(count);
    f.read(reinterpret_cast<char*>(tmp.data()), static_cast<std::streamsize>(count));
    for (size_t i = 0; i < count; ++i) a.data[i] = tmp[i];
  } else {
    throw std::runtime_error(path + ": unsupported dtype " + descr);
  }
  if (!f) throw std::runtime_error(path + ": file is shorter than its header says");
  return a;
}

}  // namespace ugv
