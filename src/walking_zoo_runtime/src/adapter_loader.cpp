#include "walking_zoo_runtime/adapter_loader.hpp"

namespace walking_zoo_runtime
{

AdapterLoader::AdapterLoader()
: class_loader_("walking_zoo_core", "walking_zoo_core::WalkingAdapter")
{
}

std::shared_ptr<walking_zoo_core::WalkingAdapter> AdapterLoader::load(
  const std::string & plugin_name)
{
  return class_loader_.createSharedInstance(plugin_name);
}

}  // namespace walking_zoo_runtime
