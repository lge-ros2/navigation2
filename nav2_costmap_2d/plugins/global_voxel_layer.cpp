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
  std::string cmd_vel_topic_string = "/cmd_vel";

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);
  std::string source;
  while (ss >> source) {
    bool init_scan_angle;
    std::string data_type;
    bool inf_is_valid;
    double publish_cycle;
    std::string cmd_vel_topic;
    std::string robot_frame_id;
    double rotate_threshold;

    declareParameter(source + "." + "init_scan_angle", rclcpp::ParameterValue(true));
    declareParameter(source + "." + "publish_cycle", rclcpp::ParameterValue(1.0));
    declareParameter(source + "." + "cmd_vel_topic", rclcpp::ParameterValue("/cmd_vel"));
    declareParameter(source + "." + "robot_frame_id", rclcpp::ParameterValue("base_link"));
    declareParameter(source + "." + "rotate_threshold", rclcpp::ParameterValue(0.17));

    node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
    node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);
    node->get_parameter(name_ + "." + source + "." + "init_scan_angle", init_scan_angle);
    node->get_parameter(name_ + "." + source + "." + "publish_cycle", publish_cycle);
    node->get_parameter(name_ + "." + source + "." + "cmd_vel_topic", cmd_vel_topic);
    node->get_parameter(name_ + "." + source + "." + "robot_frame_id", robot_frame_id);
    node->get_parameter(name_ + "." + source + "." + "rotate_threshold", rotate_threshold);

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
    cmd_vel_topic_string = cmd_vel_topic;
    rotate_threshold_ = rotate_threshold;
    robot_frame_id_ = robot_frame_id;
  }

  // [sungkyu.kang] create for obstacle index
  is_init_scan_angle_ = false;

  auto custom_qos2 = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  obstacle_grid_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "obstacle_map",
    custom_qos2);

  RCLCPP_INFO(logger_, "    GlobalVoxelLayer cmd_vel_topic: %s", cmd_vel_topic_string.c_str());
  RCLCPP_INFO(logger_, "    GlobalVoxelLayer rotate_threshold: %.2f", rotate_threshold_);
  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_string, rclcpp::SystemDefaultsQoS(),
    std::bind(&GlobalVoxelLayer::cmdVelCallback, this, std::placeholders::_1));

  scan_link_offset_ = -999.0;
}

void
GlobalVoxelLayer::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr message)
{
  last_rotate_vel_ = fabs(message->angular.z);
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

  scan_frame_id_ = message->header.frame_id;
  RCLCPP_INFO(logger_, "    scan_start_angle: %.2f, scan_end_angle_: %.2f, scan_frame_id: %s", scan_start_angle_, scan_end_angle_, scan_frame_id_.c_str());
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
  last_scan_msg_ = message;
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
  last_scan_msg_ = raw_message;
  ObstacleLayer::laserScanValidInfCallback(raw_message, buffer);
}

void GlobalVoxelLayer::updateRobotPose(const double robot_x, const double robot_y, const double robot_yaw)
{
  // [sungkyu.kang] set current_robot_pose
  current_robot_x_ = robot_x;
  current_robot_y_ = robot_y;
  current_robot_yaw_ = robot_yaw;
  // RCLCPP_INFO(logger_, "updateBounds - robot_pose: (%.2f, %.2f| %.2f)", current_robot_x_, current_robot_y_, current_robot_yaw_);
}

void GlobalVoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    updateRobotPose(robot_x, robot_y, robot_yaw);
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

  if (last_rotate_vel_ > rotate_threshold_) {
    // RCLCPP_INFO(logger_, "    do not publish obstaclesmap. last_rotate_vel_: %f, rotate_threshold_: %f", last_rotate_vel_, rotate_threshold_);
    return;
  }

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
  // RCLCPP_INFO(logger_, "GlobalVoxelLayer::prepareGrid start");
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
  std::fill(grid_->data.begin(), grid_->data.end(), -1);

  // costmap original data
  unsigned char * data = costmap.getCharMap();

  // [sungkyu.kang] check robot orientation. fill -1 back of robot.
  if (scan_link_offset_ == -999.0) {
    // initialize scan-link_offset
    auto transform_stamped = tf_->lookupTransform(
      robot_frame_id_, scan_frame_id_, tf2::TimePointZero,
      tf2::durationFromSec(0.5));

    scan_link_offset_ = transform_stamped.transform.translation.x;
    RCLCPP_INFO(logger_, "scan_link_offset: %.2f", scan_link_offset_);

  }

  // RCLCPP_INFO(logger_, "  wx: %.2f, wy: %.2f, grid_width: %d, grid_height: %d, grid_resolution: %.2f", wx, wy, grid_width, grid_height, grid_resolution);
  // RCLCPP_INFO(logger_, "  current_robot_x_: %.2f, current_robot_y_: %.2f, current_robot_yaw_: %.2f", current_robot_x_, current_robot_y_, current_robot_yaw_);

  double scan_wx = current_robot_x_ + scan_link_offset_ * cos(current_robot_yaw_);
  double scan_wy = current_robot_y_ + scan_link_offset_ * sin(current_robot_yaw_);

  int scan_mx, scan_my;
  costmap.worldToMapNoBounds(scan_wx, scan_wy, scan_mx, scan_my);

  double scan_start_angle = current_robot_yaw_ - scan_start_angle_;
  double scan_end_angle = current_robot_yaw_ - scan_end_angle_;

  double scan_check_radius = (double)grid_width / 2  * sqrt(2) * grid_resolution;
  int bf_mx, bf_my;

  // RCLCPP_INFO(logger_, "  scan_wx: %.2f, scan_wy: %.2f, scan_mx: %d, scan_my: %d, radius: %.2f, yaw: %.2f", scan_wx, scan_wy, scan_mx, scan_my, scan_check_radius, current_robot_yaw_);

  // get scan_start endpoint
  double bf_wx = scan_wx + scan_check_radius * std::cos(scan_start_angle);
  double bf_wy = scan_wy + scan_check_radius * std::sin(scan_start_angle);

  costmap.worldToMapNoBounds(bf_wx, bf_wy, bf_mx, bf_my);

  auto b_result = getBresenhamLine(scan_mx, scan_my, bf_mx, bf_my, grid_width, grid_height);
  int start_mx = b_result[b_result.size()-1][0];
  int start_my = b_result[b_result.size()-1][1];
  // RCLCPP_INFO(logger_, "    scan_start_angle: %.2f, start_mx: %d, start_my: %d", scan_start_angle, start_mx, start_my);

  // get scan_end endpoint
  bf_wx = scan_wx + scan_check_radius * std::cos(scan_end_angle);
  bf_wy = scan_wy + scan_check_radius * std::sin(scan_end_angle);

  costmap.worldToMapNoBounds(bf_wx, bf_wy, bf_mx, bf_my);

  b_result = getBresenhamLine(scan_mx, scan_my, bf_mx, bf_my, grid_width, grid_height);
  int end_mx = b_result[b_result.size()-1][0];
  int end_my = b_result[b_result.size()-1][1];
  // RCLCPP_INFO(logger_, "    scan_end_angle: %.2f, end_mx: %d, end_my: %d", scan_end_angle, end_mx, end_my);

  auto direction = SPIRAL_DIRECTION::RIGHT;
  if (start_mx == 0) {
    direction = SPIRAL_DIRECTION::UP;
  } else if (start_my == 0) {
    direction = SPIRAL_DIRECTION::LEFT;
  } else if (start_mx >= (int)grid_width -1) {
    direction = SPIRAL_DIRECTION::DOWN;
  }

  int curr_mx = start_mx;
  int curr_my = start_my;
  // RCLCPP_INFO(logger_, "      curr_mx: %d, curr_my: %d", curr_mx, curr_my);
  while(curr_mx != end_mx || curr_my != end_my) {
    // RCLCPP_INFO(logger_, "      curr_mx: %d, curr_my: %d", curr_mx, curr_my);
    fillGrid(scan_mx, scan_my, curr_mx, curr_my, grid_width, grid_height, data);

    switch (direction)
    {
      case SPIRAL_DIRECTION::RIGHT:
        if (curr_mx == (int)grid_width - 1) {
          // RCLCPP_INFO(logger_, "          change direction R -> D");
          direction = SPIRAL_DIRECTION::DOWN;
          curr_my -= 1;
        } else {
          curr_mx += 1;
        }
        break;
      case SPIRAL_DIRECTION::DOWN:
        if (curr_my == 0) {
          // RCLCPP_INFO(logger_, "          change direction D -> L");
          direction = SPIRAL_DIRECTION::LEFT;
          curr_mx -= 1;
        } else {
          curr_my -= 1;
        }
        break;
      case SPIRAL_DIRECTION::LEFT:
        if (curr_mx == 0) {
          // RCLCPP_INFO(logger_, "          change direction L -> U");
          direction = SPIRAL_DIRECTION::UP;
          curr_my += 1;
        } else {
          curr_mx -= 1;
        }
        break;
      case SPIRAL_DIRECTION::UP:
        if (curr_my == (int)grid_height - 1) {
          // RCLCPP_INFO(logger_, "          change direction U -> R");
          direction = SPIRAL_DIRECTION::RIGHT;
          curr_mx += 1;
        } else {
          curr_my += 1;
        }
        break;
    }
  }
  // fill last line
  fillGrid(scan_mx, scan_my, end_mx, end_my, grid_width, grid_height, data);

  // RCLCPP_INFO(logger_, "GlobalVoxelLayer::prepareGrid end");
}

void GlobalVoxelLayer::fillGrid(int s_x, int s_y, int f_x, int f_y, int width, int height, unsigned char * data)
{
  auto b_result = getBresenhamLine(s_x, s_y, f_x, f_y, width, height);
  std::vector<std::array<int, 2>>::iterator iter;
  bool is_obstacle = false;
  for (iter = b_result.begin(); iter != b_result.end(); iter++) {
    int x = (*iter)[0];
    int y = (*iter)[1];
    int idx = y * width + x;
    // RCLCPP_INFO(logger_, "        x: %d, y: %d, idx: %d, data: %d", x, y, idx, data[idx]);
    if (data[idx] == nav2_costmap_2d::LETHAL_OBSTACLE && !is_obstacle) {
      is_obstacle = true;
      grid_->data[idx] = 100;
      // RCLCPP_INFO(logger_, "          obstacle!!!!");
    } else if (is_obstacle) {
      grid_->data[idx] = -1;
    } else {
      grid_->data[idx] = 0;
    }
  }
}

std::vector<std::array<int, 2>> GlobalVoxelLayer::getBresenhamLine(
  int s_x, int s_y,
  int f_x, int f_y,
  int width, int height)
{
  // RCLCPP_INFO(logger_, "        getBresenhamLine s_x: %d, s_y: %d. f_x: %d, f_y: %d", s_x, s_y, f_x, f_y);
  std::vector<std::array<int, 2>> result;

  int x = s_x;
  int y = s_y;
  int W = f_x - s_x;
  int H = f_y - s_y;
  int inc_x = 1;
  if (W < 0) {
    inc_x = -1;
  } else if (W == 0) {
    inc_x = 0;
  }

  int inc_y = 1;
  if (H < 0) {
    inc_y = -1;
  } else if (H == 0) {
    inc_y = 0;
  }

  W = std::abs(W);
  H = std::abs(H);

  // 0 < 기울기 <= 1
  if ( W >= H ) {
    // 초기값
    int F = 2 * H - W; // F: 27

    // 각 판별식 공식
    int dF_1 = 2 * H;
    int dF_2 = 2 * (H - W);

    for (; x < width && y < height && x >= 0 && y >= 0; x += inc_x)
    {
      result.push_back({x, y});
      // 중단점이 0보다 작으면 그에 맞는 판별식으로 갱신, y 값은 그대로
      if (F < 0)
        F += dF_1;
      // 중단점이 0보다 크거나 같으면 그에 맞는 판별식으로 갱신, y 값 증가
      else
      {
        y += inc_y;
        F += dF_2;
      }

    }
  } else {
    // 초기값
    int F = 2 * W - H;

    // 각 판별식 공식
    int dF_1 = 2 * W;
    int dF_2 = 2 * (W - H);

    for (; x < width && y < height && x >= 0 && y >= 0; y += inc_y)
    {
      result.push_back({x, y});
      // 중단점이 0보다 작으면 그에 맞는 판별식으로 갱신, x 값은 그대로
      if (F < 0)
        F += dF_1;
      // 중단점이 0보다 크거나 같으면 그에 맞는 판별식으로 갱신, x 값 증가
      else
      {
        x += inc_x;
        F += dF_2;
      }
    }
  }
  return result;
}

}  // namespace nav2_costmap_2d
