#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "walking_zoo_runtime/footstep_planner.hpp"

namespace walking_zoo_runtime
{

// Publishes a stub FootstepPlan and matching RViz markers so the footstep
// interface is visible long before a real footstep planner exists. This node
// does not command motion; it only visualizes a deterministic plan.
class FootstepMarkerPublisher : public rclcpp::Node
{
public:
  FootstepMarkerPublisher()
  : rclcpp::Node("walking_zoo_footstep_marker_publisher")
  {
    params_.frame_id = declare_parameter<std::string>("frame_id", params_.frame_id);
    params_.start_leg = declare_parameter<std::string>("start_leg", params_.start_leg);
    params_.stride_length = declare_parameter<double>("stride_length", params_.stride_length);
    params_.stride_width = declare_parameter<double>("stride_width", params_.stride_width);
    params_.lateral_shift = declare_parameter<double>("lateral_shift", params_.lateral_shift);
    params_.swing_height = declare_parameter<double>("swing_height", params_.swing_height);
    params_.step_duration = declare_parameter<double>("step_duration", params_.step_duration);
    params_.step_count =
      static_cast<std::size_t>(declare_parameter<int>("step_count", static_cast<int>(params_.step_count)));
    const double rate = declare_parameter<double>("publish_rate", 2.0);

    plan_publisher_ =
      create_publisher<walking_zoo_msgs::msg::FootstepPlan>("/walking_zoo/footstep_plan", 10);
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/walking_zoo/footstep_markers", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(rate, 0.1));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {this->publish();});

    RCLCPP_INFO(
      get_logger(),
      "publishing %zu footstep markers in frame '%s'",
      params_.step_count, params_.frame_id.c_str());
  }

private:
  void publish()
  {
    auto plan = planner_.plan(params_);
    plan.header.stamp = now();
    for (auto & footstep : plan.footsteps) {
      footstep.header.stamp = plan.header.stamp;
    }
    plan_publisher_->publish(plan);
    marker_publisher_->publish(build_markers(plan));
  }

  visualization_msgs::msg::MarkerArray build_markers(
    const walking_zoo_msgs::msg::FootstepPlan & plan) const
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    visualization_msgs::msg::Marker path;
    path.header.frame_id = plan.frame_id;
    path.header.stamp = plan.header.stamp;
    path.ns = "footstep_path";
    path.id = 0;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action = visualization_msgs::msg::Marker::ADD;
    path.scale.x = 0.02;
    path.color.a = 0.6f;
    path.color.r = 0.6f;
    path.color.g = 0.6f;
    path.color.b = 0.6f;
    path.pose.orientation.w = 1.0;

    int id = 1;
    for (const auto & footstep : plan.footsteps) {
      visualization_msgs::msg::Marker foot;
      foot.header.frame_id = plan.frame_id;
      foot.header.stamp = plan.header.stamp;
      foot.ns = "footsteps";
      foot.id = id++;
      foot.type = visualization_msgs::msg::Marker::CUBE;
      foot.action = visualization_msgs::msg::Marker::ADD;
      foot.pose = footstep.pose;
      foot.pose.position.z += 0.015;
      foot.scale.x = 0.18;
      foot.scale.y = 0.09;
      foot.scale.z = 0.03;
      const bool is_left = footstep.leg_name == "left";
      foot.color.a = 0.9f;
      foot.color.r = is_left ? 0.30f : 0.30f;
      foot.color.g = is_left ? 0.60f : 0.90f;
      foot.color.b = is_left ? 1.00f : 0.55f;
      markers.markers.push_back(foot);

      path.points.push_back(footstep.pose.position);
    }

    markers.markers.push_back(path);
    return markers;
  }

  FootstepPlanner planner_;
  FootstepPlanParams params_;
  rclcpp::Publisher<walking_zoo_msgs::msg::FootstepPlan>::SharedPtr plan_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace walking_zoo_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<walking_zoo_runtime::FootstepMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
