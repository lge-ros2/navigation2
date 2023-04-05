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

#include "nav2_behavior_tree/plugins/action/print_blackboard_double.hpp"

namespace nav2_behavior_tree
{

PrintBlackboardDouble::PrintBlackboardDouble(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus PrintBlackboardDouble::tick()
{
  getInput("value", value_);
  std::cout << "PrintBlackboardDouble - value: " << value_.data << std::endl;

  return BT::NodeStatus::SUCCESS;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PrintBlackboardDouble>("PrintBlackboardDouble");
}
