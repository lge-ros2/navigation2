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

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

CheckFrontObstacle::CheckFrontObstacle(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::CoroActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");

  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");
}

BT::NodeStatus CheckFrontObstacle::tick()
{

  setStatus(BT::NodeStatus::RUNNING);

  geometry_msgs::msg::PoseStamped pose;
  double angular_distance_weight;
  double distance_forward;

  getInput("angular_distance_weight", angular_distance_weight);
  getInput("distance_forward", distance_forward);
  RCLCPP_DEBUG(node_->get_logger(), "[CheckFrontObstacle] distance_forward: %.2f", distance_forward);

  nav_msgs::msg::Path new_path;
  getInput("path", new_path);
  if (!new_path.poses.empty()) {
    if (new_path != path_) {
      path_ = new_path;
    }
  } else {
    RCLCPP_INFO(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "CheckRobotOrientation::tick new_path.poses.empty()");
  }
  closest_pose_detection_begin_ = path_.poses.begin();

  if (!getRobotPose(path_.header.frame_id, pose)) {
    RCLCPP_ERROR(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "canot get robot pose");
    return BT::NodeStatus::FAILURE;
  }

  auto closest_pose_detection_end = path_.poses.end();
  closest_pose_detection_end = nav2_util::geometry_utils::first_after_integrated_distance(
    closest_pose_detection_begin_, path_.poses.end(), std::numeric_limits<double>::infinity());

  // find the closest pose on the path
  auto current_pose = nav2_util::geometry_utils::min_by(
    closest_pose_detection_begin_, closest_pose_detection_end,
    [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
      return poseDistance(pose, ps, angular_distance_weight);
    });

  closest_pose_detection_begin_ = current_pose;

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

  if (output_path.poses.size() < 2) {
    return BT::NodeStatus::SUCCESS;
  }

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  request->path = output_path;
  auto result = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->is_valid) {
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;

}

inline bool CheckFrontObstacle::getRobotPose(
  std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose)
{
  if (!getInput("pose", pose)) {
    std::string robot_frame;
    if (!getInput("robot_frame", robot_frame)) {
      RCLCPP_ERROR(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Neither pose nor robot_frame specified for %s", name().c_str());
      return false;
    }
    double transform_tolerance;
    getInput("transform_tolerance", transform_tolerance);
    if (!nav2_util::getCurrentPose(
        pose, *tf_buffer_, path_frame_id, robot_frame, transform_tolerance))
    {
      RCLCPP_WARN(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Failed to lookup current robot pose for %s", name().c_str());
      return false;
    }
  }
  return true;
}

double CheckFrontObstacle::poseDistance(
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


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CheckFrontObstacle>("CheckFrontObstacle");
}
