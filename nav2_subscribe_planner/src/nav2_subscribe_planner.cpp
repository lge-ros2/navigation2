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
#include "nav2_util/geometry_utils.hpp"
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
    logger_, "Destroying plugin %s of type SubscribePlanner",
    name_.c_str());
}

void
SubscribePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  node_ = parent;
  name_ = name;

  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
    logger_, "Configuring plugin %s of type SubscribePlanner",
    name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node, name + ".plan_topic", rclcpp::ParameterValue("fms_plan"));
  node->get_parameter(name + ".plan_topic", topic_name_);

  declare_parameter_if_not_declared(node, name + ".path_pruning", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".path_pruning", path_pruning_);

  auto callback = [this](const nav_msgs::msg::Path::SharedPtr msg) -> void {
    path_ = *msg;
  };

  plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    topic_name_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
    callback);
}

void
SubscribePlanner::activate()
{
  RCLCPP_INFO(
    logger_, "Activating plugin %s of type SubscribePlanner",
    name_.c_str());
}

void
SubscribePlanner::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating plugin %s of type SubscribePlanner",
    name_.c_str());
}

void
SubscribePlanner::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type SubscribePlanner",
    name_.c_str());

  plan_sub_.reset();
}

nav_msgs::msg::Path SubscribePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & /* goal */)
{

  if (path_.poses.empty()) {
    return path_;
  }

  double angular_distance_weight = 0.0;
  double max_robot_pose_search_dist = std::numeric_limits<double>::infinity();
  double distance_forward = std::numeric_limits<double>::infinity();


  closest_pose_detection_begin_ = path_.poses.begin();
  auto closest_pose_detection_end = path_.poses.end();

  if (path_pruning_) {
    closest_pose_detection_end = nav2_util::geometry_utils::first_after_integrated_distance(
      closest_pose_detection_begin_, path_.poses.end(), max_robot_pose_search_dist);
  }


  // find the closest pose on the path
  auto current_pose = nav2_util::geometry_utils::min_by(
    closest_pose_detection_begin_, closest_pose_detection_end,
    [&start, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
      return poseDistance(start, ps, angular_distance_weight);
    });

  if (path_pruning_) {
    closest_pose_detection_begin_ = current_pose;
  }

  // expand forwards to extract desired length
  auto forward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
    current_pose, path_.poses.end(), distance_forward);
  // expand backwards to extract desired length
  // Note: current_pose + 1 is used because reverse iterator points to a cell before it
  auto backward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
    std::reverse_iterator(current_pose + 1), path_.poses.rend(), 0.0);
  nav_msgs::msg::Path output_path;
  output_path.header = path_.header;
  output_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(
    backward_pose_it.base(), forward_pose_it);
  return output_path;
}

double
SubscribePlanner::poseDistance(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2,
  const double angular_distance_weight)
{
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  // taking angular distance into account in addition to spatial distance
  // (to improve picking a correct pose near cusps and loops)
  tf2::Quaternion q1;
  tf2::convert(pose1.pose.orientation, q1);
  tf2::Quaternion q2;
  tf2::convert(pose2.pose.orientation, q2);
  double da = angular_distance_weight * std::abs(q1.angleShortestPath(q2));
  return std::sqrt(dx * dx + dy * dy + da * da);
}


}  // namespace nav2_subscribe_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_subscribe_planner::SubscribePlanner, nav2_core::GlobalPlanner)
