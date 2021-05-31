#include "nav2_others_footprint_costmap_plugin/others_footprint_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_others_footprint_costmap_plugin
{

OthersFootprintLayer::OthersFootprintLayer()
{
}

void OthersFootprintLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + "." + "enabled", enabled_);
  declareParameter("topic", rclcpp::ParameterValue("/lge_seocho/B1F/others_footprint"));
  node_->get_parameter(name_ + "." + "topic", topic_);
  declareParameter("time_tolerance", rclcpp::ParameterValue(3.0));
  node_->get_parameter(name_ + "." + "time_tolerance", time_tolerance_);

  std::string full_ns = node_->get_namespace();
  size_t split = full_ns.find('/');
  if (split == std::string::npos) {
    device_id_ = full_ns;
  } else {
    std::string without_first_slash = full_ns;
    if(split == 0) {
      without_first_slash = full_ns.substr(1);
    }
    split = without_first_slash.find('/');
    device_id_ = without_first_slash.substr(0, split);
  }

  auto callback = [this](const geometry_msgs::msg::PolygonStamped::SharedPtr msg) -> void {
    if(device_id_ == "cloi2") {
      return;
    }
    std::string frame_id = msg->header.frame_id;
    if(frame_id != device_id_ && frame_id != "") {
      if(footprint_map_.find(frame_id) != footprint_map_.end()) {
        footprint_map_[frame_id] = msg;
      } else {
        footprint_map_.insert(make_pair(frame_id, msg));
      }
    }
  };
  footprint_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
    topic_,
    rclcpp::SensorDataQoS(),
    callback);

  RCLCPP_INFO(rclcpp::get_logger(
    "nav2_costmap_2d"), "OthersFootprintLayer enabled: %d",
    enabled_);
  RCLCPP_INFO(rclcpp::get_logger(
    "nav2_costmap_2d"), "OthersFootprintLayer topic: %s",
    topic_.c_str());
  RCLCPP_INFO(rclcpp::get_logger(
    "nav2_costmap_2d"), "OthersFootprintLayer time_tolerance: %f",
    time_tolerance_);
}

void OthersFootprintLayer::updateBounds(
  double /* robot_x */, double /* robot_y */, double /* robot_yaw */, 
  double * /* min_x */, double * /* min_y */, double * /* max_x */, double * /* max_y */)
{
}

void OthersFootprintLayer::onFootprintChanged()
{
  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "OthersFootprintLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

void OthersFootprintLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, 
  int /* min_i */, int /* min_j */, int /* max_i */, int /* max_j */)
{
  if (!enabled_) {
    return;
  }
  unsigned char * master_array = master_grid.getCharMap();
  auto system_clock = node_->get_clock();
  bool ros_time_is_active = system_clock->ros_time_is_active();
  rclcpp::Time system_now = system_clock->now();
  rclcpp::Duration tolerance_duration = rclcpp::Duration(time_tolerance_, 0);
  for(auto it=footprint_map_.begin(); it!=footprint_map_.end(); ++it) {
    geometry_msgs::msg::PolygonStamped::SharedPtr footprint = it->second;
    rclcpp::Time msg_time = footprint->header.stamp;
    rclcpp::Duration time_sub = system_now - msg_time;
    if(time_sub < tolerance_duration) {
      for(unsigned int point_index = 0; point_index < footprint->polygon.points.size(); point_index++) {
        double wx = (double)footprint->polygon.points[point_index].x;
        double wy = (double)footprint->polygon.points[point_index].y;
        int mx, my;
        master_grid.worldToMapNoBounds(wx, wy, mx, my);
        int index = master_grid.getIndex(mx, my);
        master_array[index] = LETHAL_OBSTACLE;
      }
    }
  }

}

}  // namespace nav2_others_footprint_costmap_plugin

// This is the macro allowing a nav2_others_footprint_costmap_plugin::OthersFootprintLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_others_footprint_costmap_plugin::OthersFootprintLayer, nav2_costmap_2d::Layer)
