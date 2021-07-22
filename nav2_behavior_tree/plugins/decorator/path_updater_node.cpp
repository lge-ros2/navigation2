// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/path_updater_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

PathUpdater::PathUpdater(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string path_updater_topic;
  node_->get_parameter_or<std::string>("path_updater_topic", path_updater_topic, "plan");

  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    path_updater_topic, 10, std::bind(&PathUpdater::callback_updated_path, this, _1));
}

inline BT::NodeStatus PathUpdater::tick()
{
  RCLCPP_INFO(node_->get_logger(), "PathUpdater::tick start");
  nav_msgs::msg::Path path;
  // getInput("input_path", path);
  // if (rclcpp::Time(last_path_received_.header.stamp) > rclcpp::Time(path.header.stamp)) {
  //   path = last_path_received_;
  // }
  path = last_path_received_;
  setOutput("output_path", path);
  if (path.poses.size() != 0) {
    geometry_msgs::msg::PoseStamped final_pose = path.poses.back();
    RCLCPP_INFO(node_->get_logger(), "PathUpdater::tick output_path final_pose: %f, %f", 
      final_pose.pose.position.x, final_pose.pose.position.y);
  } else {
    RCLCPP_INFO(node_->get_logger(), "PathUpdater::tick output_path size 0");
  }
  return child_node_->executeTick();
}

void
PathUpdater::callback_updated_path(const nav_msgs::msg::Path::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped final_pose = msg->poses.back();
  RCLCPP_INFO(node_->get_logger(), "PathUpdater::callback_updated_path final_pose: %f, %f", 
    final_pose.pose.position.x, final_pose.pose.position.y);
  last_path_received_ = *msg;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathUpdater>("PathUpdater");
}
