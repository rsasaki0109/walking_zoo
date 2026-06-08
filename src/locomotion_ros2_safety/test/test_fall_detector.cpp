#include <gtest/gtest.h>

#include "locomotion_ros2_safety/fall_detector.hpp"

using locomotion_ros2_safety::FallDetector;
using locomotion_ros2_safety::FallState;

TEST(FallDetector, UprightWhenLevel)
{
  FallDetector detector;
  EXPECT_EQ(detector.classify(0.0, 0.0), FallState::UPRIGHT);
  EXPECT_FALSE(detector.is_fallen(0.0, 0.05));
}

TEST(FallDetector, TiltedInWarnBand)
{
  FallDetector detector(0.35, 0.70);
  // ~0.4 rad pitch is past the warn threshold but short of a fall.
  EXPECT_EQ(detector.classify(0.0, 0.4), FallState::TILTED);
  EXPECT_FALSE(detector.is_fallen(0.0, 0.4));
}

TEST(FallDetector, FallenWhenPastFallThreshold)
{
  FallDetector detector(0.35, 0.70);
  EXPECT_EQ(detector.classify(0.0, 0.9), FallState::FALLEN);
  EXPECT_TRUE(detector.is_fallen(0.0, 0.9));
}

TEST(FallDetector, CombinesRollAndPitch)
{
  FallDetector detector(0.35, 0.70);
  // Each axis alone is upright, but combined tilt magnitude crosses the warn band.
  EXPECT_EQ(detector.classify(0.05, 0.05), FallState::UPRIGHT);
  EXPECT_EQ(detector.classify(0.3, 0.3), FallState::TILTED);
}

TEST(FallDetector, IsSymmetricInSign)
{
  FallDetector detector(0.35, 0.70);
  EXPECT_EQ(detector.classify(0.0, 0.9), detector.classify(0.0, -0.9));
  EXPECT_EQ(detector.classify(0.5, 0.0), detector.classify(-0.5, 0.0));
}

TEST(FallDetector, OrdersThresholdsRegardlessOfArgumentOrder)
{
  // Warn and fall passed in the wrong order are reordered internally.
  FallDetector detector(0.70, 0.35);
  EXPECT_LE(detector.tilt_warn_rad(), detector.tilt_fall_rad());
  EXPECT_EQ(detector.classify(0.0, 0.9), FallState::FALLEN);
}
