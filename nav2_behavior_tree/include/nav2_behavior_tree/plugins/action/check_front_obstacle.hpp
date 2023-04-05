// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_FRONT_OBSTACLE_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_FRONT_OBSTACLE_NODE_HPP_

#include <memory>
#include <string>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

class CheckFrontObstacle : public BT::CoroActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PlannerSelector
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   */
  CheckFrontObstacle(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<float>(
        "angle",
        0.4,
        "the input angle radian to detect front obstacle"),
      BT::InputPort<float>(
        "range",
        2.0,
        "the input range meter to detect front obstacle"),
      BT::InputPort<float>(
        "rotate_threshold",
        0.3,
        "the input threshold rotation velocity to ignore range"),
    };
  }

private:
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief callback function for detecting obstacle
   *
   * @param msg the message with the laser scan data
   */
  void callbackLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  /**
   * @brief callback function for check rotation velcity
   *
   * @param msg the message with the cmd vel data
   */
  void callbackCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  float angle_;
  float range_;
  float rotate_threshold_;

  sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;
  geometry_msgs::msg::Twist::SharedPtr last_cmd_vel_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_FRONT_OBSTACLE_NODE_HPP_
