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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_BLACKBOARD_DOUBLE_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_BLACKBOARD_DOUBLE_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/float32.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

class PrintBlackboardDouble : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PrintBlackboardDouble
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   */
  PrintBlackboardDouble(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std_msgs::msg::Float32>(
        "value",
        "the input to print value."),
    };
  }

private:
  std_msgs::msg::Float32 value_;

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_BLACKBOARD_DOUBLE_NODE_HPP_
