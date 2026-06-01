#ifndef WALKING_ZOO_RUNTIME__ADAPTER_LOADER_HPP_
#define WALKING_ZOO_RUNTIME__ADAPTER_LOADER_HPP_

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "walking_zoo_core/walking_adapter.hpp"

namespace walking_zoo_runtime
{

class AdapterLoader
{
public:
  AdapterLoader();

  std::shared_ptr<walking_zoo_core::WalkingAdapter> load(const std::string & plugin_name);

private:
  pluginlib::ClassLoader<walking_zoo_core::WalkingAdapter> class_loader_;
};

}  // namespace walking_zoo_runtime

#endif  // WALKING_ZOO_RUNTIME__ADAPTER_LOADER_HPP_
