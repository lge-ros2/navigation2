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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REQUEST_TRIGGER_LPP_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REQUEST_TRIGGER_LPP_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

class RequestTriggerLpp : public BT::CoroActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PlannerSelector
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   */
  RequestTriggerLpp(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "request_topic_name",
        "request_lpp",
        "the input topic name to request lpp"),
      BT::InputPort<std::string>(
        "status_topic_name",
        "lpp_status",
        "the input topic name to publish status"),
      BT::InputPort<std::string>(
        "trigger_topic_name",
        "trigger_lpp",
        "the input topic name to tigger lpp"),
      BT::InputPort<nav_msgs::msg::Path>(
        "path",
        "path from fms"),
      BT::InputPort<std::string>(
        "robot_frame", "base_link",
        "Robot base frame id"),
      BT::InputPort<double>(
        "transform_tolerance", 0.2,
        "Transform lookup tolerance"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "pose", "Manually specified pose to be used"
        "if overriding current robot pose"),        
      BT::OutputPort<unsigned int>(
        "lpp_duration",
        "the output duration to lpp"),
    };
  }

private:
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief callback function for the trigger lpp topic
   *
   * @param msg the message with the device_id for trigger lpp
   */
  void callbackTriggerLpp(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr trigger_sub_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  /**
   * @brief Get either specified input pose or robot pose in path frame
   * @param path_frame_id Frame ID of path
   * @param pose Output pose
   * @return True if succeeded
   */
  bool getRobotPose(std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose);

  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  bool send_request_;
  std::string request_topic_name_;
  std::string status_topic_name_;
  std::string trigger_topic_name_;
  std::string device_id_;

  float last_trigger_duration_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REQUEST_TRIGGER_LPP_NODE_HPP_
