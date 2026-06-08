#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace locomotion_ros2_gait_lab_sil
{

/** Dependency-free loader + MLP inference for gait_lab ``rl_policy*.npz`` exports. */
class GaitLabRlPolicy
{
public:
  bool load(const std::string & path, std::string * error = nullptr);

  std::size_t observation_dim() const {return obs_mean_.size();}

  std::vector<double> infer(const std::vector<double> & observation) const;

private:
  std::vector<double> obs_mean_;
  std::vector<double> obs_std_;
  std::vector<std::vector<double>> weights_;
  std::vector<std::vector<double>> biases_;
};

}  // namespace locomotion_ros2_gait_lab_sil
