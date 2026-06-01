#ifndef WALKING_ZOO_VLA__SEMANTIC_ACTION_MAPPER_HPP_
#define WALKING_ZOO_VLA__SEMANTIC_ACTION_MAPPER_HPP_

#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "walking_zoo_msgs/msg/semantic_action.hpp"

namespace walking_zoo_vla
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
  SemanticMapping map(const walking_zoo_msgs::msg::SemanticAction & action) const;
};

}  // namespace walking_zoo_vla

#endif  // WALKING_ZOO_VLA__SEMANTIC_ACTION_MAPPER_HPP_
