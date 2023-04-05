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

#include "nav2_costmap_2d/global_voxel_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::GlobalVoxelLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{
char * GlobalVoxelLayer::cost_translation_table_ = NULL;

void GlobalVoxelLayer::onInitialize()
{
  VoxelLayer::onInitialize();

  RCLCPP_INFO(logger_, "[GlobalVoxelLayer] onInitialize with Obstacle Publisher");

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // The topics that we'll subscribe to from the parameter server
  std::string topics_string;
  node->get_parameter(name_ + "." + "observation_sources", topics_string);

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);
  std::string source;
  while (ss >> source) {
    bool init_scan_angle;
    std::string data_type;
    bool inf_is_valid;
    double publish_cycle;
    declareParameter(source + "." + "init_scan_angle", rclcpp::ParameterValue(true));
    declareParameter(source + "." + "scan_link_offset", rclcpp::ParameterValue(2.0));
    declareParameter(source + "." + "publish_cycle", rclcpp::ParameterValue(1.0));

    node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
    node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);
    node->get_parameter(name_ + "." + source + "." + "init_scan_angle", init_scan_angle);
    node->get_parameter(name_ + "." + source + "." + "scan_link_offset", scan_link_offset_);
    node->get_parameter(name_ + "." + source + "." + "publish_cycle", publish_cycle);

    if (publish_cycle > 0) {
      rclcpp::Duration::from_seconds(publish_cycle);
    } else {
      publish_cycle_ = rclcpp::Duration(0, 0);
    }

    // create a callback for the topic
    if (data_type == "LaserScan") {
      auto filter = std::dynamic_pointer_cast<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(observation_notifiers_.back());
      if (inf_is_valid) {
        filter->registerCallback(
          std::bind(
            &GlobalVoxelLayer::laserScanValidInfCallback, this, std::placeholders::_1,
            observation_buffers_.back()));

      } else {
        filter->registerCallback(
          std::bind(
            &GlobalVoxelLayer::laserScanCallback, this, std::placeholders::_1,
            observation_buffers_.back()));
      }
    }

    // [sungkyu.kang] initialize use_init_scan_angle_ parameter
    use_init_scan_angle_ = init_scan_angle;
  }

  // [sungkyu.kang] create for obstacle index
  is_init_scan_angle_ = false;

  auto custom_qos2 = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  obstacle_grid_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "obstacle_map",
    custom_qos2);

  if (cost_translation_table_ == NULL) {
    cost_translation_table_ = new char[256];

    // special values:
    cost_translation_table_[0] = FREE_SPACE;  // NO obstacle
    cost_translation_table_[253] = 99;  // INSCRIBED obstacle
    cost_translation_table_[254] = 100;  // LETHAL obstacle
    cost_translation_table_[255] = -1;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++) {
      cost_translation_table_[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
    }
  }
}

void
GlobalVoxelLayer::initializeScanAngle(sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
  // [sungkyu.kang] initialize scan angles
  RCLCPP_INFO(logger_, "GlobalVoxelLayer::initializeScanAngle");
  if (use_init_scan_angle_) {
    float current_angle = message->angle_min;
    float range = 0.0;

    for (size_t i = 0; i < message->ranges.size(); i++) {
      range = message->ranges[i];
      if (range >= message->range_min && range <= message->range_max) {
        scan_start_angle_ = current_angle + 0.05;
        break;
      }
      current_angle = current_angle + message->angle_increment;
    }
    current_angle = message->angle_max;
    for (size_t i = message->ranges.size(); i > 0; i--) {
      range = message->ranges[i];
      if (range >= message->range_min && range <= message->range_max) {
        scan_end_angle_ = current_angle - 0.05;
        break;
      }
      current_angle = current_angle - message->angle_increment;
    }
  } else {
    scan_start_angle_ = message->angle_min;
    scan_end_angle_ = message->angle_max;
  }

  RCLCPP_INFO(logger_, "    scan_start_angle: %f, scan_end_angle_: %f", scan_start_angle_, scan_end_angle_);
  is_init_scan_angle_ = true;
}

void
GlobalVoxelLayer::laserScanCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr message,
  const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer)
{
  if (!is_init_scan_angle_) {
    initializeScanAngle(message);
  }
  ObstacleLayer::laserScanCallback(message, buffer);
}

void
GlobalVoxelLayer::laserScanValidInfCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr raw_message,
  const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer)
{
  if (!is_init_scan_angle_) {
    initializeScanAngle(raw_message);
  }
  laserScanValidInfCallback(raw_message, buffer);
}

void GlobalVoxelLayer::updateRobotYaw(const double robot_yaw)
{
  // [sungkyu.kang] set current_robot_yaw_
  current_robot_yaw_ = robot_yaw;
  // RCLCPP_INFO(logger_, "updateBounds - robot_yaw: %f, current_robot_yaw_: %f", robot_yaw, current_robot_yaw_);
}

void GlobalVoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    updateRobotYaw(robot_yaw);
  }

  VoxelLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void GlobalVoxelLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j)
{
  ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);

  // [sungkyu.kang] publish current master grid
  if (publish_cycle_ > rclcpp::Duration(0, 0) && obstacle_grid_pub_->get_subscription_count() > 0) {
    const auto current_time = clock_->now();
    if ((last_publish_ + publish_cycle_ < current_time) ||  // publish_cycle_ is due
      (current_time < last_publish_))      // time has moved backwards, probably due to a switch to sim_time // NOLINT
    {
      prepareGrid(master_grid);
      obstacle_grid_pub_->publish(std::move(grid_));
      last_publish_ = current_time;
    }
  }
}

void GlobalVoxelLayer::prepareGrid(nav2_costmap_2d::Costmap2D costmap)
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap.getMutex()));
  float grid_resolution = costmap.getResolution();
  unsigned int grid_width = costmap.getSizeInCellsX();
  unsigned int grid_height = costmap.getSizeInCellsY();

  grid_ = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  grid_->header.frame_id = global_frame_;
  grid_->header.stamp = clock_->now();

  grid_->info.resolution = grid_resolution;

  grid_->info.width = grid_width;
  grid_->info.height = grid_height;

  double wx, wy;
  costmap.mapToWorld(0, 0, wx, wy);
  grid_->info.origin.position.x = wx - grid_resolution / 2;
  grid_->info.origin.position.y = wy - grid_resolution / 2;
  grid_->info.origin.position.z = 0.0;
  grid_->info.origin.orientation.w = 1.0;

  grid_->data.resize(grid_->info.width * grid_->info.height);

  unsigned char * data = costmap.getCharMap();

  double normalLeftX = -1 * sin(current_robot_yaw_ - scan_start_angle_);
  double normalLeftY = cos(current_robot_yaw_ - scan_start_angle_);
  double normalRightX = sin(current_robot_yaw_ - scan_end_angle_);
  double normalRightY = -1 * cos(current_robot_yaw_ - scan_end_angle_);

  // RCLCPP_INFO(
  //   logger_,
  //   "\nGlobalVoxelLayer::prepareGrid %f, %f, %f.\n    %f, %f, %f, %f", current_robot_yaw_, current_robot_yaw_ - scan_start_angle_, current_robot_yaw_ - scan_end_angle_, normalLeftX, normalLeftY, normalRightX, normalRightY);
  int x, y;
  unsigned int row_index, col_index;
  int scan_tf_offset_x = round(scan_link_offset_ * cos(current_robot_yaw_));
  int scan_tf_offset_y = round(scan_link_offset_ * sin(current_robot_yaw_));
  // RCLCPP_INFO(logger_, "scan_tf_offset_x: %d, scan_tf_offset_y: %d", scan_tf_offset_x, scan_tf_offset_y);
  for (unsigned int i = 0; i < grid_->data.size(); i++) {
    // [sungkyu.kang] check robot orientation. fill -1 back of robot.
    row_index = i / grid_width;
    col_index = i % grid_width;
    x = col_index - grid_height / 2 - scan_tf_offset_x;
    y = row_index - grid_width / 2 - scan_tf_offset_y;
    // RCLCPP_INFO(
    //   logger_,
    //   " [%4d] row_index: %2d, col_index: %2d, (%2d, %2d)", i, row_index, col_index, x, y);
    if (x * normalLeftX + y * normalLeftY > 0 && x * normalRightX + y * normalRightY > 0) {
      // RCLCPP_INFO( logger_,"    hit!!");
      grid_->data[i] = -1;
    } else {
      grid_->data[i] = cost_translation_table_[data[i]];
    }
  }
  // RCLCPP_INFO(logger_, "GlobalVoxelLayer::prepareGrid finish ");
}
}  // namespace nav2_costmap_2d
