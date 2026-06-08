#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "gtest/gtest.h"
#include "locomotion_ros2_gait_lab_sil/gait_lab_rl_policy.hpp"

namespace
{

std::string policy_path(const char * filename)
{
  const char * root = std::getenv("LOCOMOTION_ROS2_GAIT_LAB_PATH");
  if (root == nullptr) {
    return {};
  }
  return (std::filesystem::path(root) / "gait_lab" / filename).string();
}

}  // namespace

TEST(GaitLabRlPolicy, loads_residual_policy)
{
  const auto path = policy_path("rl_policy.npz");
  if (path.empty() || !std::filesystem::exists(path)) {
    GTEST_SKIP() << "rl_policy.npz not available";
  }
  locomotion_ros2_gait_lab_sil::GaitLabRlPolicy policy;
  std::string error;
  ASSERT_TRUE(policy.load(path, &error)) << error;
  EXPECT_EQ(policy.observation_dim(), 34u);
  std::vector<double> obs(34, 0.0);
  const auto action = policy.infer(obs);
  ASSERT_EQ(action.size(), 12u);
}

TEST(GaitLabRlPolicy, loads_steerable_policy)
{
  const auto path = policy_path("rl_policy_steer.npz");
  if (path.empty() || !std::filesystem::exists(path)) {
    GTEST_SKIP() << "rl_policy_steer.npz not available";
  }
  locomotion_ros2_gait_lab_sil::GaitLabRlPolicy policy;
  std::string error;
  ASSERT_TRUE(policy.load(path, &error)) << error;
  EXPECT_EQ(policy.observation_dim(), 36u);
}
