#ifndef LOCOMOTION_ROS2_VLA__SEMANTIC_ACTION_MAPPER_HPP_
#define LOCOMOTION_ROS2_VLA__SEMANTIC_ACTION_MAPPER_HPP_

#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "locomotion_ros2_msgs/msg/semantic_action.hpp"

namespace locomotion_ros2_vla
{

struct SemanticMapping
{
  bool recognized{false};
  bool stop{false};
  geometry_msgs::msg::TwistStamped velocity;
  std::string status_text;
};

class SemanticActionMapper
{
public:
  SemanticMapping map(const locomotion_ros2_msgs::msg::SemanticAction & action) const;
};

}  // namespace locomotion_ros2_vla

#endif  // LOCOMOTION_ROS2_VLA__SEMANTIC_ACTION_MAPPER_HPP_
