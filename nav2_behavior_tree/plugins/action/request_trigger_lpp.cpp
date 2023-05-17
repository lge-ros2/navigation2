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

#include "nav2_behavior_tree/plugins/action/request_trigger_lpp.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

RequestTriggerLpp::RequestTriggerLpp(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::CoroActionNode(name, conf)
{
  last_trigger_duration_ = 0.0;
  send_request_ = false;
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  getInput("request_topic_name", request_topic_name_);
  getInput("status_topic_name", status_topic_name_);
  getInput("trigger_topic_name", trigger_topic_name_);

  request_pub_ = node_->create_publisher<std_msgs::msg::String>(request_topic_name_, 5);
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(status_topic_name_, 5);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  trigger_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    trigger_topic_name_,
    qos,
    std::bind(&RequestTriggerLpp::callbackTriggerLpp, this, _1),
    sub_option);
}

BT::NodeStatus RequestTriggerLpp::tick()
{
  if (!send_request_) {
    last_trigger_duration_ = 0.0;
    RCLCPP_INFO(node_->get_logger(), "RequestTriggerLpp send_request_ topic: %s", request_topic_name_.c_str());
    auto request_message = std_msgs::msg::String();
    request_message.data = "request";
    request_pub_->publish(request_message);
    send_request_ = true;
  }

  callback_group_executor_.spin_some();

  // This behavior always use the last selected planner received from the topic input.
  // When no input is specified it uses the default planner.
  // If the default planner is not specified then we work in "required planner mode":
  // In this mode, the behavior returns failure if the planner selection is not received from
  // the topic input.
  if (last_trigger_duration_ == 0.0) {
    return BT::NodeStatus::RUNNING;
  } else {
    // std_msgs::msg::Float32 new_duration;
    // new_duration.data = last_trigger_duration_;
    setOutput("lpp_duration", (unsigned int)(last_trigger_duration_*1000));
    RCLCPP_INFO(node_->get_logger(), "RequestTriggerLpp lpp_duration: %f", last_trigger_duration_);
    nav_msgs::msg::Path path_;
    geometry_msgs::msg::PoseStamped pose_;
    getInput("path", path_);

    auto status_message = std_msgs::msg::String();
    status_message.data = "start";
    status_pub_->publish(status_message);
    last_trigger_duration_ = 0.0;
    send_request_ = false;
    return BT::NodeStatus::SUCCESS;
  }
}

void
RequestTriggerLpp::callbackTriggerLpp(const std_msgs::msg::Float32::SharedPtr msg)
{
  last_trigger_duration_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RequestTriggerLpp>("RequestTriggerLpp");
}
