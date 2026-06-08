#include "locomotion_ros2_gait_lab_sil/gait_lab_rl_policy.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace locomotion_ros2_gait_lab_sil
{
namespace
{

struct NpyArray
{
  std::vector<std::size_t> shape;
  std::vector<double> data;
};

bool read_u16_le(const std::vector<std::uint8_t> & buf, std::size_t offset, std::uint16_t & out)
{
  if (offset + 2 > buf.size()) {
    return false;
  }
  out = static_cast<std::uint16_t>(buf[offset]) |
    (static_cast<std::uint16_t>(buf[offset + 1]) << 8);
  return true;
}

bool read_u32_le(const std::vector<std::uint8_t> & buf, std::size_t offset, std::uint32_t & out)
{
  if (offset + 4 > buf.size()) {
    return false;
  }
  out = static_cast<std::uint32_t>(buf[offset]) |
    (static_cast<std::uint32_t>(buf[offset + 1]) << 8) |
    (static_cast<std::uint32_t>(buf[offset + 2]) << 16) |
    (static_cast<std::uint32_t>(buf[offset + 3]) << 24);
  return true;
}

std::size_t product(const std::vector<std::size_t> & shape)
{
  std::size_t n = 1;
  for (const auto dim : shape) {
    n *= dim;
  }
  return n;
}

bool parse_npy(const std::vector<std::uint8_t> & raw, NpyArray & out, std::string & error)
{
  if (raw.size() < 10 || std::memcmp(raw.data(), "\x93NUMPY", 6) != 0) {
    error = "invalid npy magic";
    return false;
  }
  const auto major = raw[6];
  const auto minor = raw[7];
  std::size_t header_len = 0;
  std::size_t header_start = 0;
  if (major == 1 && minor == 0) {
    std::uint16_t len16 = 0;
    if (!read_u16_le(raw, 8, len16)) {
      error = "truncated npy header";
      return false;
    }
    header_len = len16;
    header_start = 10;
  } else if (major == 2 && minor == 0) {
    std::uint32_t len32 = 0;
    if (!read_u32_le(raw, 8, len32)) {
      error = "truncated npy header";
      return false;
    }
    header_len = len32;
    header_start = 12;
  } else {
    error = "unsupported npy version";
    return false;
  }
  if (header_start + header_len > raw.size()) {
    error = "truncated npy header payload";
    return false;
  }
  const std::string header(
    reinterpret_cast<const char *>(raw.data() + header_start), header_len);
  const auto descr_pos = header.find("'descr':");
  const auto shape_pos = header.find("'shape':");
  if (descr_pos == std::string::npos || shape_pos == std::string::npos) {
    error = "missing npy header fields";
    return false;
  }
  const auto descr_quote = header.find('\'', descr_pos + 8);
  const auto descr_end = header.find('\'', descr_quote + 1);
  if (descr_quote == std::string::npos || descr_end == std::string::npos) {
    error = "invalid descr field";
    return false;
  }
  const std::string descr = header.substr(descr_quote + 1, descr_end - descr_quote - 1);
  const auto paren_open = header.find('(', shape_pos);
  const auto paren_close = header.find(')', paren_open);
  if (paren_open == std::string::npos || paren_close == std::string::npos) {
    error = "invalid shape field";
    return false;
  }
  std::vector<std::size_t> shape;
  std::stringstream shape_stream(header.substr(paren_open + 1, paren_close - paren_open - 1));
  std::string token;
  while (std::getline(shape_stream, token, ',')) {
    token.erase(token.begin(), std::find_if(token.begin(), token.end(), [](unsigned char c) {
        return !std::isspace(c);
      }));
    token.erase(
      std::find_if(token.rbegin(), token.rend(), [](unsigned char c) {
        return !std::isspace(c);
      }).base(),
      token.end());
    if (token.empty()) {
      continue;
    }
    shape.push_back(static_cast<std::size_t>(std::stoull(token)));
  }
  const std::size_t count = product(shape);
  const std::size_t data_offset = header_start + header_len;
  if (data_offset + count * sizeof(double) > raw.size() && descr != "<f4" && descr != "<f8") {
    error = "truncated npy payload";
    return false;
  }
  out.shape = shape;
  out.data.resize(count);
  if (descr == "<f8") {
    if (data_offset + count * sizeof(double) > raw.size()) {
      error = "truncated f8 payload";
      return false;
    }
    for (std::size_t i = 0; i < count; ++i) {
      double v = 0.0;
      std::memcpy(&v, raw.data() + data_offset + i * sizeof(double), sizeof(double));
      out.data[i] = v;
    }
  } else if (descr == "<f4") {
    if (data_offset + count * sizeof(float) > raw.size()) {
      error = "truncated f4 payload";
      return false;
    }
    for (std::size_t i = 0; i < count; ++i) {
      float v = 0.0f;
      std::memcpy(&v, raw.data() + data_offset + i * sizeof(float), sizeof(float));
      out.data[i] = static_cast<double>(v);
    }
  } else if (descr == "<i8") {
    if (data_offset + count * sizeof(std::int64_t) > raw.size()) {
      error = "truncated i8 payload";
      return false;
    }
    for (std::size_t i = 0; i < count; ++i) {
      std::int64_t v = 0;
      std::memcpy(&v, raw.data() + data_offset + i * sizeof(std::int64_t), sizeof(std::int64_t));
      out.data[i] = static_cast<double>(v);
    }
  } else {
    error = "unsupported descr: " + descr;
    return false;
  }
  return true;
}

bool load_npz_arrays(
  const std::string & path, std::unordered_map<std::string, NpyArray> & arrays,
  std::string & error)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    error = "could not open policy file: " + path;
    return false;
  }
  std::vector<std::uint8_t> zip(
    (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  std::size_t offset = 0;
  while (offset + 30 < zip.size()) {
    if (zip[offset] != 0x50 || zip[offset + 1] != 0x4b || zip[offset + 2] != 0x03 ||
      zip[offset + 3] != 0x04)
    {
      break;
    }
    std::uint16_t method = 0;
    std::uint32_t comp_size = 0;
    std::uint16_t name_len = 0;
    std::uint16_t extra_len = 0;
    if (!read_u16_le(zip, offset + 8, method) ||
      !read_u32_le(zip, offset + 18, comp_size) ||
      !read_u16_le(zip, offset + 26, name_len) ||
      !read_u16_le(zip, offset + 28, extra_len))
    {
      error = "truncated zip local header";
      return false;
    }
    const std::size_t name_start = offset + 30;
    const std::size_t extra_start = name_start + name_len;
    if (name_start + name_len > zip.size() || extra_start + extra_len > zip.size()) {
      error = "truncated zip entry name/extra";
      return false;
    }
    if (comp_size == 0xffffffffu && extra_len >= 4) {
      std::size_t extra_offset = extra_start;
      while (extra_offset + 4 <= extra_start + extra_len) {
        std::uint16_t header_id = 0;
        std::uint16_t data_size = 0;
        if (!read_u16_le(zip, extra_offset, header_id) ||
          !read_u16_le(zip, extra_offset + 2, data_size))
        {
          break;
        }
        extra_offset += 4;
        if (extra_offset + data_size > extra_start + extra_len) {
          break;
        }
        if (header_id == 0x0001 && data_size >= 16) {
          std::uint64_t uncompressed = 0;
          std::uint64_t compressed = 0;
          std::memcpy(&uncompressed, zip.data() + extra_offset, sizeof(uncompressed));
          std::memcpy(&compressed, zip.data() + extra_offset + 8, sizeof(compressed));
          comp_size = static_cast<std::uint32_t>(compressed);
        }
        extra_offset += data_size;
      }
    }
    const std::size_t data_start = extra_start + extra_len;
    if (data_start + comp_size > zip.size()) {
      error = "truncated zip entry";
      return false;
    }
    const std::string entry_name(
      reinterpret_cast<const char *>(zip.data() + name_start), name_len);
    if (method != 0) {
      error = "compressed npz entries are not supported";
      return false;
    }
    NpyArray arr;
    std::string npy_error;
    const std::vector<std::uint8_t> npy_raw(
      zip.begin() + static_cast<std::ptrdiff_t>(data_start),
      zip.begin() + static_cast<std::ptrdiff_t>(data_start + comp_size));
    if (!parse_npy(npy_raw, arr, npy_error)) {
      error = "failed to parse " + entry_name + ": " + npy_error;
      return false;
    }
    arrays[entry_name] = std::move(arr);
    offset = data_start + comp_size;
  }
  if (arrays.empty()) {
    error = "no arrays found in npz";
    return false;
  }
  return true;
}

std::vector<double> matvec(
  const std::vector<double> & matrix, std::size_t rows, std::size_t cols,
  const std::vector<double> & vec, const std::vector<double> & bias)
{
  std::vector<double> out(rows, 0.0);
  for (std::size_t r = 0; r < rows; ++r) {
    double acc = (r < bias.size()) ? bias[r] : 0.0;
    for (std::size_t c = 0; c < cols; ++c) {
      acc += matrix[r * cols + c] * vec[c];
    }
    out[r] = acc;
  }
  return out;
}

}  // namespace

bool GaitLabRlPolicy::load(const std::string & path, std::string * error)
{
  std::string local_error;
  std::unordered_map<std::string, NpyArray> arrays;
  if (!load_npz_arrays(path, arrays, local_error)) {
    if (error) {
      *error = local_error;
    }
    return false;
  }
  const auto layers_it = arrays.find("n_layers.npy");
  if (layers_it == arrays.end() || layers_it->second.data.empty()) {
    if (error) {
      *error = "missing n_layers.npy";
    }
    return false;
  }
  const int n_layers = static_cast<int>(layers_it->second.data[0]);
  weights_.clear();
  biases_.clear();
  weights_.reserve(static_cast<std::size_t>(n_layers));
  biases_.reserve(static_cast<std::size_t>(n_layers));
  for (int i = 0; i < n_layers; ++i) {
    const std::string w_name = "W" + std::to_string(i) + ".npy";
    const std::string b_name = "b" + std::to_string(i) + ".npy";
    const auto w_it = arrays.find(w_name);
    const auto b_it = arrays.find(b_name);
    if (w_it == arrays.end() || b_it == arrays.end() || w_it->second.shape.size() != 2) {
      if (error) {
        *error = "missing layer weights: " + w_name;
      }
      return false;
    }
    weights_.push_back(w_it->second.data);
    biases_.push_back(b_it->second.data);
  }
  const auto mean_it = arrays.find("obs_mean.npy");
  const auto std_it = arrays.find("obs_std.npy");
  if (mean_it == arrays.end() || std_it == arrays.end()) {
    if (error) {
      *error = "missing observation normaliser arrays";
    }
    return false;
  }
  obs_mean_ = mean_it->second.data;
  obs_std_ = std_it->second.data;
  if (obs_mean_.size() != obs_std_.size() || obs_mean_.empty()) {
    if (error) {
      *error = "invalid observation normaliser size";
    }
    return false;
  }
  return true;
}

std::vector<double> GaitLabRlPolicy::infer(const std::vector<double> & observation) const
{
  if (observation.size() != obs_mean_.size() || weights_.empty()) {
    return {};
  }
  std::vector<double> h(observation.size());
  for (std::size_t i = 0; i < observation.size(); ++i) {
    const double denom = (obs_std_[i] == 0.0) ? 1.0 : obs_std_[i];
    h[i] = (observation[i] - obs_mean_[i]) / denom;
  }
  for (std::size_t layer = 0; layer < weights_.size(); ++layer) {
    const std::size_t rows = weights_[layer].size() / h.size();
    const std::size_t cols = h.size();
    h = matvec(weights_[layer], rows, cols, h, biases_[layer]);
    if (layer + 1 < weights_.size()) {
      for (double & v : h) {
        v = std::tanh(v);
      }
    }
  }
  return h;
}

}  // namespace locomotion_ros2_gait_lab_sil
