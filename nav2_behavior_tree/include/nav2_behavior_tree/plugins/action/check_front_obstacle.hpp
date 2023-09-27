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

#include "behaviortree_cpp_v3/action_node.h"
#include "tf2_ros/buffer.h"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"

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
      BT::InputPort<nav_msgs::msg::Path>("path", "Original Path"),
      BT::InputPort<std::string>(
        "robot_frame", "base_link",
        "Robot base frame id"),
      BT::InputPort<double>(
        "distance_forward", 1.5,
        "forward distance for check orientation of path"),
      BT::InputPort<double>(
        "transform_tolerance", 0.2,
        "Transform lookup tolerance"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "pose", "Manually specified pose to be used"
        "if overriding current robot pose"),
      BT::InputPort<double>(
        "angular_distance_weight", 0.0,
        "Weight of angular distance relative to positional distance when finding which path "
        "pose is closest to robot. Not applicable on paths without orientations assigned"),
    };
  }

private:
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Get either specified input pose or robot pose in path frame
   * @param path_frame_id Frame ID of path
   * @param pose Output pose
   * @return True if succeeded
   */
  bool getRobotPose(std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief A custom pose distance method which takes angular distance into account
   * in addition to spatial distance (to improve picking a correct pose near cusps and loops)
   * @param pose1 Distance is computed between this pose and pose2
   * @param pose2 Distance is computed between this pose and pose1
   * @param angular_distance_weight Weight of angular distance relative to spatial distance
   * (1.0 means that 1 radian of angular distance corresponds to 1 meter of spatial distance)
   */
  static double poseDistance(
    const geometry_msgs::msg::PoseStamped & pose1,
    const geometry_msgs::msg::PoseStamped & pose2,
    const double angular_distance_weight);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path::_poses_type::iterator closest_pose_detection_begin_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
  // The timeout value while waiting for a responce from the
  // is path valid service
  std::chrono::milliseconds server_timeout_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_FRONT_OBSTACLE_NODE_HPP_
