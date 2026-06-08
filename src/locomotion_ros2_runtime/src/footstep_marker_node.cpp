#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "locomotion_ros2_runtime/footstep_planner.hpp"
#include "locomotion_ros2_runtime/occupancy_terrain.hpp"
#include "locomotion_ros2_runtime/step_feasibility_checker.hpp"
#include "locomotion_ros2_runtime/terrain_model.hpp"

namespace locomotion_ros2_runtime
{

// Publishes a stub FootstepPlan and matching RViz markers so the footstep
// interface is visible long before a real footstep planner exists. This node
// does not command motion; it only visualizes a deterministic plan.
class FootstepMarkerPublisher : public rclcpp::Node
{
public:
  FootstepMarkerPublisher()
  : rclcpp::Node("locomotion_ros2_footstep_marker_publisher")
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

    // Optional terrain so the preview can show feet dodging a keep-out zone and
    // stepping up onto a curb. A keep-out zone is [min_x, min_y, max_x, max_y];
    // a curb is [min_x, min_y, max_x, max_y, height]. Empty disables them.
    const auto no_step_zone =
      declare_parameter<std::vector<double>>("no_step_zone", std::vector<double>{});
    const auto curb_box =
      declare_parameter<std::vector<double>>("curb_box", std::vector<double>{});
    if (no_step_zone.size() == 4) {
      locomotion_ros2_runtime::TerrainBox box;
      box.min_x = no_step_zone[0];
      box.min_y = no_step_zone[1];
      box.max_x = no_step_zone[2];
      box.max_y = no_step_zone[3];
      box.no_step = true;
      terrain_.add_box(box);
    }
    if (curb_box.size() == 5) {
      locomotion_ros2_runtime::TerrainBox box;
      box.min_x = curb_box[0];
      box.min_y = curb_box[1];
      box.max_x = curb_box[2];
      box.max_y = curb_box[3];
      box.height = curb_box[4];
      terrain_.add_box(box);
    }

    // Real terrain source: a Nav2-style costmap drives the keep-out zones and an
    // optional elevation grid drives the step-up heights. Empty topic disables
    // the subscription and keeps the hand-authored boxes above as the only
    // terrain (so the existing flat/box demo path is unchanged).
    occupied_threshold_ = static_cast<std::int8_t>(
      declare_parameter<int>("occupied_threshold", 50));
    unknown_is_no_step_ = declare_parameter<bool>("unknown_is_no_step", false);
    elevation_height_per_unit_ =
      declare_parameter<double>("elevation_height_per_unit", 0.0015);
    const auto costmap_topic = declare_parameter<std::string>("costmap_topic", "");
    const auto elevation_topic = declare_parameter<std::string>("elevation_topic", "");

    if (!costmap_topic.empty()) {
      costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic, rclcpp::QoS(1).transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {this->on_costmap(*msg);});
      RCLCPP_INFO(get_logger(), "terrain costmap source: '%s'", costmap_topic.c_str());
    }
    if (!elevation_topic.empty()) {
      elevation_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        elevation_topic, rclcpp::QoS(1).transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {this->on_elevation(*msg);});
      RCLCPP_INFO(get_logger(), "terrain elevation source: '%s'", elevation_topic.c_str());
    }

    plan_publisher_ =
      create_publisher<locomotion_ros2_msgs::msg::FootstepPlan>("/locomotion_ros2/footstep_plan", 10);
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/locomotion_ros2/footstep_markers", 10);

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
  void on_costmap(const nav_msgs::msg::OccupancyGrid & msg)
  {
    costmap_grid_ = locomotion_ros2_runtime::terrain_grid_from_costmap(
      msg, occupied_threshold_, unknown_is_no_step_);
    have_costmap_ = true;
    // Plan in the costmap frame so foothold queries align with the map cells.
    if (!msg.header.frame_id.empty()) {
      params_.frame_id = msg.header.frame_id;
    }
    rebuild_terrain();
    RCLCPP_INFO_ONCE(
      get_logger(), "terrain costmap received: %ux%u @ %.3fm in frame '%s'",
      msg.info.width, msg.info.height, msg.info.resolution, params_.frame_id.c_str());
  }

  void on_elevation(const nav_msgs::msg::OccupancyGrid & msg)
  {
    elevation_msg_ = msg;
    have_elevation_ = true;
    rebuild_terrain();
  }

  void rebuild_terrain()
  {
    if (!have_costmap_) {
      return;
    }
    locomotion_ros2_runtime::TerrainGrid grid = costmap_grid_;
    if (have_elevation_) {
      locomotion_ros2_runtime::set_elevation_from_grid(
        grid, elevation_msg_, elevation_height_per_unit_);
    }
    terrain_.set_grid(grid);
  }

  void publish()
  {
    const auto terrain_plan = planner_.plan_over_terrain(params_, terrain_);
    auto plan = terrain_plan.plan;
    plan.header.stamp = now();
    for (auto & footstep : plan.footsteps) {
      footstep.header.stamp = plan.header.stamp;
    }
    plan_publisher_->publish(plan);
    marker_publisher_->publish(build_markers(plan, terrain_plan.placements));
  }

  visualization_msgs::msg::MarkerArray build_markers(
    const locomotion_ros2_msgs::msg::FootstepPlan & plan,
    const std::vector<locomotion_ros2_runtime::StepPlacement> & placements) const
  {
    const auto feasibility = feasibility_checker_.evaluate(plan, StepFeasibilityLimits{});

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
    std::size_t index = 0;
    for (const auto & footstep : plan.footsteps) {
      const bool feasible =
        index >= feasibility.steps.size() || feasibility.steps[index].feasible;
      const bool blocked = index < placements.size() && placements[index].blocked;
      const bool adjusted = index < placements.size() && placements[index].adjusted;
      ++index;

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
      if (blocked || !feasible) {
        // Red: no clear foothold, or the feasibility gate rejected the step
        // (e.g. the swing needed to clear a curb exceeds the limit).
        foot.color.r = 0.95f;
        foot.color.g = 0.25f;
        foot.color.b = 0.25f;
      } else if (adjusted) {
        // Amber: foot was nudged sideways to dodge a keep-out zone.
        foot.color.r = 0.95f;
        foot.color.g = 0.75f;
        foot.color.b = 0.10f;
      } else {
        foot.color.r = 0.30f;
        foot.color.g = is_left ? 0.60f : 0.90f;
        foot.color.b = is_left ? 1.00f : 0.55f;
      }
      markers.markers.push_back(foot);

      path.points.push_back(footstep.pose.position);
    }

    markers.markers.push_back(path);
    return markers;
  }

  FootstepPlanner planner_;
  StepFeasibilityChecker feasibility_checker_;
  FootstepPlanParams params_;
  TerrainModel terrain_;

  std::int8_t occupied_threshold_{50};
  bool unknown_is_no_step_{false};
  double elevation_height_per_unit_{0.0015};
  locomotion_ros2_runtime::TerrainGrid costmap_grid_;
  bool have_costmap_{false};
  nav_msgs::msg::OccupancyGrid elevation_msg_;
  bool have_elevation_{false};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr elevation_sub_;
  rclcpp::Publisher<locomotion_ros2_msgs::msg::FootstepPlan>::SharedPtr plan_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace locomotion_ros2_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<locomotion_ros2_runtime::FootstepMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
