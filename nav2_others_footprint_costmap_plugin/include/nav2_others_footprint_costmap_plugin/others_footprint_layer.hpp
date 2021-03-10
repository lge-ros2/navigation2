#ifndef NAV2_OTHERS_FOOTPRINT_LAYER_HPP_
#define NAV2_OTHERS_FOOTPRINT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"


namespace nav2_others_footprint_costmap_plugin
{

class OthersFootprintLayer : public nav2_costmap_2d::Layer
{
public:
  OthersFootprintLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, 
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}
private:
  std::string device_id_;
  std::string topic_;
  double time_tolerance_;

  std::map<std::string, geometry_msgs::msg::PolygonStamped::SharedPtr> footprint_map_;

  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
};

}  // namespace nav2_others_footprint_costmap_plugin

#endif  // NAV2_OTHERS_FOOTPRINT_LAYER_HPP_
