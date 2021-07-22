// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
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

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

// #define BENCHMARK_TESTING

#include "nav2_subscribe_planner/nav2_subscribe_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;

namespace nav2_subscribe_planner
{

SubscribePlanner::SubscribePlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

SubscribePlanner::~SubscribePlanner()
{
  RCLCPP_INFO(
    node_->get_logger(), "Destroying plugin %s of type SubscribePlanner",
    name_.c_str());
}

void
SubscribePlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  node_ = parent;
  name_ = name;

  RCLCPP_INFO(
    node_->get_logger(), "Configuring plugin %s of type SubscribePlanner",
    name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node_, name + ".topic", rclcpp::ParameterValue("plan"));
  node_->get_parameter(name + ".topic", topic_name_);

  auto callback = [this](const nav_msgs::msg::Path::SharedPtr msg) -> void {
    path_ = *msg;
  };

  plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    topic_name_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
    callback);
}

void
SubscribePlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type SubscribePlanner",
    name_.c_str());
}

void
SubscribePlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type SubscribePlanner",
    name_.c_str());
}

void
SubscribePlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "Cleaning up plugin %s of type SubscribePlanner",
    name_.c_str());

  plan_sub_.reset();
}

nav_msgs::msg::Path SubscribePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & /*start*/,
  const geometry_msgs::msg::PoseStamped & /*goal*/)
{
  return path_;
}

}  // namespace nav2_subscribe_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_subscribe_planner::SubscribePlanner, nav2_core::GlobalPlanner)
