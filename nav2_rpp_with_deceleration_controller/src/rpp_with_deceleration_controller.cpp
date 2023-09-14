// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_rpp_with_deceleration_controller/rpp_with_deceleration_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
// #include "nav2_util/geometry_utils.hpp"
// #include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

// using std::hypot;
// using std::min;
// using std::max;
// using std::abs;
// using nav2_util::declare_parameter_if_not_declared;
// using nav2_util::geometry_utils::euclidean_distance;
// using namespace nav2_costmap_2d;  // NOLINT
// using rcl_interfaces::msg::ParameterType;

namespace nav2_rpp_with_deceleration_controller
{

RPPWithDecelerationController::RPPWithDecelerationController() 
{

}

void RPPWithDecelerationController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".slowing_angle_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_decel", rclcpp::ParameterValue(3.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_angular_velocity", rclcpp::ParameterValue(0.2));

  node->get_parameter(plugin_name_ + ".slowing_angle_distance", slowing_angle_distance_);
  node->get_parameter(plugin_name_ + ".max_angular_decel", max_angular_decel_);
  node->get_parameter(plugin_name_ + ".min_angular_velocity", min_angular_velocity_);
  RCLCPP_INFO(logger_,"    param slowing_angle_distance: %.2f", slowing_angle_distance_);
  RCLCPP_INFO(logger_,"    param max_angular_decel: %.2f", max_angular_decel_);
  RCLCPP_INFO(logger_,"    param min_angular_velocity: %.2f", min_angular_velocity_);
}

geometry_msgs::msg::TwistStamped RPPWithDecelerationController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{

  auto cmd_vel = RegulatedPurePursuitController::computeVelocityCommands(pose, speed, goal_checker);

  std::lock_guard<std::mutex> lock_reinit(mutex_);

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (allow_reversing_) {
    // Cusp check
    double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  linear_vel = desired_linear_vel_;

  // Make sure we're in compliance with basic constraints
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    if (fabs(angle_to_goal) < slowing_angle_distance_) {
      rotateToHeadingWithDeceleration(linear_vel, angular_vel, angle_to_goal, speed);
      cmd_vel.twist.angular.z = angular_vel;
    }
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    if (fabs(angle_to_heading) < slowing_angle_distance_) {
      rotateToHeadingWithDeceleration(linear_vel, angular_vel, angle_to_heading, speed);
      cmd_vel.twist.angular.z = angular_vel;
    }
  }
  // RCLCPP_INFO(logger_,"              result angular_vel:  %.2f", cmd_vel.twist.angular.z);
  return cmd_vel;
}

void RPPWithDecelerationController::rotateToHeadingWithDeceleration(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // RCLCPP_INFO(logger_,"  ----  RPPWithDecelerationController::rotateToHeadingWithDeceleration angle_to_path: %.2f", angle_to_path);
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * rotate_to_heading_angular_vel_;

  const double & dt = control_duration_;
  double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
  double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;

  min_feasible_angular_speed = sign * min_angular_velocity_;
  max_feasible_angular_speed = curr_speed.angular.z - sign * max_angular_decel_ * dt;
  if (min_feasible_angular_speed < max_feasible_angular_speed) {
    if (sign == -1) {
      max_feasible_angular_speed = min_feasible_angular_speed;
    }
    // RCLCPP_INFO(logger_,"              curr: %.2f, min: %.2f, max: %.2f, sign: %.2f", curr_speed.angular.z, min_feasible_angular_speed, max_feasible_angular_speed, sign);
    angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
  } else {
    angular_vel = std::clamp(angular_vel, max_feasible_angular_speed, min_feasible_angular_speed);
    // RCLCPP_INFO(logger_,"              curr: %.2f, min: %.2f, max: %.2f, sign: %.2f", curr_speed.angular.z, max_feasible_angular_speed, min_feasible_angular_speed, sign);
  }
}

}  // namespace nav2_rpp_with_deceleration_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_rpp_with_deceleration_controller::RPPWithDecelerationController,
  nav2_core::Controller)
