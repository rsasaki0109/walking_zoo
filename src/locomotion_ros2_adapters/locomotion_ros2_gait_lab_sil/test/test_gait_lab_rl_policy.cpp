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

TEST(GaitLabRlPolicy, matches_python_reference_on_zero_observation)
{
  const auto path = policy_path("rl_policy.npz");
  if (path.empty() || !std::filesystem::exists(path)) {
    GTEST_SKIP() << "rl_policy.npz not available";
  }
  locomotion_ros2_gait_lab_sil::GaitLabRlPolicy policy;
  std::string error;
  ASSERT_TRUE(policy.load(path, &error)) << error;

  // Golden action from gait_lab Python ``_policy`` on a zero 34-dim observation.
  const std::vector<double> obs(34, 0.0);
  const std::vector<double> expected = {
    -0.068544572310642371, 0.38329460713620334, 0.21760907832120671,
    0.12700651988180367, 0.56734049201423198, -0.55464092856475278,
    -0.20856410597612801, 0.31230267170429843, -0.14121563616676761,
    0.16051338957264405, 0.33410490083451833, 0.17985213187481236,
  };
  const auto action = policy.infer(obs);
  ASSERT_EQ(action.size(), expected.size());
  for (std::size_t i = 0; i < action.size(); ++i) {
    EXPECT_NEAR(action[i], expected[i], 1e-5);
  }
}
