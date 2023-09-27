// Copyright (c) 2023 LG Electronics. All rights reserved.
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

#ifndef NAV2_RPP_WITH_DECELERATION_CONTROLLER_HPP_
#define NAV2_RPP_WITH_DECELERATION_CONTROLLER_HPP_

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

namespace nav2_rpp_with_deceleration_controller
{

/**
 * @class nav2_rpp_with_deceleration_controller::RPPWithDecelerationController
 * @brief Regulated pure pursuit controller with deceleration plugin
 */
class RPPWithDecelerationController : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::RPPWithDecelerationController
   */
  RPPWithDecelerationController();

  /**
   * @brief Destrructor for nav2_regulated_pure_pursuit_controller::RPPWithDecelerationController
   */
  ~RPPWithDecelerationController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

protected:

  double slowing_angle_distance_;
  double max_angular_decel_;
  double min_angular_velocity_;

  void rotateToHeadingWithDeceleration(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);

  rclcpp::Logger logger_ {rclcpp::get_logger("RPPWithDecelerationController")};

};

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_RPP_WITH_DECELERATION_CONTROLLER_HPP_
