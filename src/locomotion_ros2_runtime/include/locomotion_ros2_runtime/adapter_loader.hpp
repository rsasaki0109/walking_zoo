#ifndef LOCOMOTION_ROS2_RUNTIME__ADAPTER_LOADER_HPP_
#define LOCOMOTION_ROS2_RUNTIME__ADAPTER_LOADER_HPP_

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "locomotion_ros2_core/walking_adapter.hpp"

namespace locomotion_ros2_runtime
{

class AdapterLoader
{
public:
  AdapterLoader();

  std::shared_ptr<locomotion_ros2_core::WalkingAdapter> load(const std::string & plugin_name);

private:
  pluginlib::ClassLoader<locomotion_ros2_core::WalkingAdapter> class_loader_;
};

}  // namespace locomotion_ros2_runtime

#endif  // LOCOMOTION_ROS2_RUNTIME__ADAPTER_LOADER_HPP_
