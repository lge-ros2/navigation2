/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_critics/rotate_to_path.hpp"
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"

PLUGINLIB_EXPORT_CLASS(dwb_critics::RotateToPathCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{
void RotateToPathCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  path_yaw_tolerance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".path_yaw_tolerance", 0.05);

  max_idx_global_plan_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".max_idx_global_plan", 10);

  need_to_rotate_ = false;

  reset();
}

void RotateToPathCritic::reset()
{
}

bool RotateToPathCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &global_plan)
{
  if (fabs(vel.x)<0.01 && fabs(vel.theta)<0.01)
  {
    need_to_rotate_ = true;
    const auto idx = std::min(static_cast<int>(global_plan.poses.size() - 1), max_idx_global_plan_);
    const auto& other_pose = global_plan.poses[idx];
    path_yaw_ = std::atan2(other_pose.y - pose.y, other_pose.x - pose.x);
    // std::cout << "[YJ] Robot needs to rotate!!! " <<  path_yaw_ <<std::endl;
    // std::cout << "[YJ] RotateToPathCritic::prepare vel.x=" << vel.x << ", vel.theta = " << vel.theta << std::endl;
  }
  return true;
}

double RotateToPathCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (need_to_rotate_ == false)
    return 0.0;
  else
  {
    if (fabs(traj.velocity.x) > 0 || fabs(traj.velocity.y) > 0)
      throw dwb_core::IllegalTrajectoryException(name_, "Non-rotation command when the robot heading is off.");
    return scoreRotation(traj);
  }
}

double RotateToPathCritic::scoreRotation(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (traj.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory.");
  }

  const auto end_yaw = traj.poses.back().theta;
  const auto diff_yaw = fabs(angles::shortest_angular_distance(end_yaw, path_yaw_));
  // std::cout << "[YJ] " << end_yaw << "," << path_yaw_ << std::endl;
  if (diff_yaw < path_yaw_tolerance_) {
    // std::cout << "[YJ] Rotated to Path!!! " << diff_yaw << "<" << path_yaw_tolerance_ << std::endl;
    need_to_rotate_ = false;
  }
  return diff_yaw;
}

}  // namespace dwb_critics