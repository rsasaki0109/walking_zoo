#include "locomotion_ros2_runtime/adapter_loader.hpp"

namespace locomotion_ros2_runtime
{

AdapterLoader::AdapterLoader()
: class_loader_("locomotion_ros2_core", "locomotion_ros2_core::WalkingAdapter")
{
}

std::shared_ptr<locomotion_ros2_core::WalkingAdapter> AdapterLoader::load(
  const std::string & plugin_name)
{
  return class_loader_.createSharedInstance(plugin_name);
}

}  // namespace locomotion_ros2_runtime
