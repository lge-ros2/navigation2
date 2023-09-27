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

#include "nav2_behavior_tree/plugins/action/wait_for_start_lpp.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

WaitForStartLpp::WaitForStartLpp(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::CoroActionNode(name, conf)
{
  start_flag_ = -1;
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  getInput("start_topic_name", start_topic_name_);
  getInput("status_topic_name", status_topic_name_);

  RCLCPP_INFO(node_->get_logger(), "WaitForStartLpp start_topic_name: %s", start_topic_name_.c_str());
  RCLCPP_INFO(node_->get_logger(), "WaitForStartLpp status_topic_name: %s", status_topic_name_.c_str());

  status_pub_ = node_->create_publisher<std_msgs::msg::String>(status_topic_name_, 5);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  start_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    start_topic_name_,
    qos,
    std::bind(&WaitForStartLpp::callbackStartLpp, this, _1),
    sub_option);
}

BT::NodeStatus WaitForStartLpp::tick()
{
  callback_group_executor_.spin_some();

  if (start_flag_ == -1) {
    return BT::NodeStatus::RUNNING;
  } else {
    if (start_flag_ == 0) {
      start_flag_ = -1;
      return BT::NodeStatus::FAILURE;
    }

    auto status_message = std_msgs::msg::String();
    status_message.data = "start";
    status_pub_->publish(status_message);
    start_flag_ = -1;
    return BT::NodeStatus::SUCCESS;
  }
}

void
WaitForStartLpp::callbackStartLpp(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "WaitForStartLpp callbackStartLpp msg->data: %d", msg->data);
  if (msg->data) {
    start_flag_ = 1;
  } else {
    start_flag_ = 0; 
  }
  RCLCPP_INFO(node_->get_logger(), "WaitForStartLpp callbackStartLpp start_flag_: %d", start_flag_);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::WaitForStartLpp>("WaitForStartLpp");
}
