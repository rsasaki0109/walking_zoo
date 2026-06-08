#include <iostream>
#include <string>
#include <vector>

#include "locomotion_ros2_gait_lab_sil/gait_lab_rl_policy.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "usage: gait_lab_rl_policy_infer <policy.npz> <obs0> <obs1> ...\n";
    return 2;
  }

  locomotion_ros2_gait_lab_sil::GaitLabRlPolicy policy;
  std::string error;
  if (!policy.load(argv[1], &error)) {
    std::cerr << "failed to load policy: " << error << "\n";
    return 1;
  }

  std::vector<double> observation;
  observation.reserve(static_cast<std::size_t>(argc - 2));
  for (int i = 2; i < argc; ++i) {
    observation.push_back(std::stod(argv[i]));
  }

  const auto action = policy.infer(observation);
  if (action.empty()) {
    std::cerr << "inference failed: observation dim " << observation.size()
              << " expected " << policy.observation_dim() << "\n";
    return 1;
  }

  for (std::size_t i = 0; i < action.size(); ++i) {
    if (i > 0) {
      std::cout << ' ';
    }
    std::cout << action[i];
  }
  std::cout << '\n';
  return 0;
}
