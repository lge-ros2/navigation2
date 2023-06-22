/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__GLOBAL_VOXEL_LAYER_HPP_
#define NAV2_COSTMAP_2D__GLOBAL_VOXEL_LAYER_HPP_

#include "nav2_costmap_2d/voxel_layer.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_costmap_2d
{
/**
 * @class GlobalVoxelLayer
 * @brief Takes laser and pointcloud data to populate a 3D voxel representation of the environment
 */
class GlobalVoxelLayer : public VoxelLayer
{
public:
  /**
   * @brief Voxel Layer constructor
   */
  GlobalVoxelLayer()
  : VoxelLayer()
  {};
  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y);

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief  A callback to handle buffering LaserScan messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void laserScanCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer);

  /**
   * @brief A callback to handle buffering LaserScan messages which need filtering to turn Inf values into range_max.
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void laserScanValidInfCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer);

  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr message);

protected:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_grid_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> grid_;

  double current_robot_x_;
  double current_robot_y_;
  double current_robot_yaw_;
  std::string scan_frame_id_;
  std::string robot_frame_id_;
  bool is_init_scan_angle_;
  bool use_init_scan_angle_;
  double scan_start_angle_;
  double scan_end_angle_;
  float scan_link_offset_;
  double last_rotate_vel_;
  double rotate_threshold_;
  static char * cost_translation_table_;
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration publish_cycle_{1, 0};
  sensor_msgs::msg::LaserScan::ConstSharedPtr last_scan_msg_;

protected:
  void updateRobotPose(const double robot_x, const double robot_y, const double robot_yaw);

private:
  void initializeScanAngle(sensor_msgs::msg::LaserScan::ConstSharedPtr message);
  void prepareGrid(nav2_costmap_2d::Costmap2D costmap);
  std::vector<std::array<int, 2>> bresenham(int s_x, int s_y, int f_x, int f_y, int width, int height);
  void fillGrid(int s_x, int s_y, int f_x, int f_y, int width, int height, unsigned char * data);

  enum SPIRAL_DIRECTION
  {
    RIGHT = 0,
    DOWN = 1,
    LEFT = 2,
    UP = 3
  };
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__VOXEL_LAYER_HPP_
