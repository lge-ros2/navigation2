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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/check_front_obstacle.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

CheckFrontObstacle::CheckFrontObstacle(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::CoroActionNode(name, conf)
{
  last_scan_msg_ = std::make_shared<sensor_msgs::msg::LaserScan>();
  last_cmd_vel_ = std::make_shared<geometry_msgs::msg::Twist>();

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  getInput("angle", angle_);
  getInput("range", range_);
  getInput("rotate_threshold", rotate_threshold_);

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  
  rclcpp::QoS scan_qos(rclcpp::KeepLast(1));
  scan_qos.best_effort().durability_volatile();

  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan",
    scan_qos,
    std::bind(&CheckFrontObstacle::callbackLaserScan, this, _1),
    sub_option);

  rclcpp::QoS cmd_vel_qos(rclcpp::KeepLast(1));
  cmd_vel_qos.durability_volatile();    
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    cmd_vel_qos,
    std::bind(&CheckFrontObstacle::callbackCmdVel, this, _1),
    sub_option);
}

BT::NodeStatus CheckFrontObstacle::tick()
{

  callback_group_executor_.spin_some();

  // uninitialized scan data
  if (last_scan_msg_->angle_increment == 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "[CheckFrontObstacle] scan msg is not initialized. return SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  if (last_cmd_vel_->angular.z < -1 * rotate_threshold_ || last_cmd_vel_->angular.z > rotate_threshold_) {
    RCLCPP_DEBUG(node_->get_logger(), "[CheckFrontObstacle] rotation_velocity: %.2f. return SUCCESS", last_cmd_vel_->angular.z);
    return BT::NodeStatus::SUCCESS;
  }

  float angle_min = last_scan_msg_->angle_min;
  float angle_increment = last_scan_msg_->angle_increment;
  for (size_t i = 0; i < last_scan_msg_->ranges.size(); i++) {
    float range = last_scan_msg_->ranges[i];
    float angle = angle_min + angle_increment * i;
    if (angle > -1 * angle_ / 2 && angle < angle_ /2) {
      if (range > 0 && range < range_) {
        RCLCPP_INFO(node_->get_logger(), "[CheckFrontObstacle] angle: %.2f, range: %.2f. rotation_vel: %.2f return FAILURE", angle, range, last_cmd_vel_->angular.z);
        return BT::NodeStatus::FAILURE;
      }
    }
  }
  RCLCPP_DEBUG(node_->get_logger(), "[CheckFrontObstacle] cannot find obstacles. return SUCCESS. angle_: %.2f, range: %.2f", angle_, range_);
  return BT::NodeStatus::SUCCESS;

}

void
CheckFrontObstacle::callbackLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_msg_ = msg;
}


void 
CheckFrontObstacle::callbackCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_vel_ = msg;
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CheckFrontObstacle>("CheckFrontObstacle");
}
