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
 *         Steve Macenski
 *********************************************************************/
#include "nav2_costmap_2d/obstacle_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

char * ObstacleLayer::cost_translation_table_ = NULL;

ObstacleLayer::~ObstacleLayer()
{
  dyn_params_handler_.reset();
  for (auto & notifier : observation_notifiers_) {
    notifier.reset();
  }
}

void ObstacleLayer::onInitialize()
{
  RCLCPP_INFO(logger_, "[ObstacleLayer] onInitialize with Obstacle Index Publisher");
  bool track_unknown_space;
  double transform_tolerance;

  // The topics that we'll subscribe to from the parameter server
  std::string topics_string;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.0));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + "." + "min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter("track_unknown_space", track_unknown_space);
  node->get_parameter("transform_tolerance", transform_tolerance);
  node->get_parameter(name_ + "." + "observation_sources", topics_string);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ObstacleLayer::dynamicParametersCallback,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "Subscribed to Topics: %s", topics_string.c_str());

  rolling_window_ = layered_costmap_->isRolling();

  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }

  ObstacleLayer::matchSize();
  current_ = true;
  was_reset_ = false;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source) {
    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking, init_scan_angle;

    declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
    declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(std::string("")));
    declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "data_type", rclcpp::ParameterValue(std::string("LaserScan")));
    declareParameter(source + "." + "min_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));
    declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "obstacle_max_range", rclcpp::ParameterValue(2.5));
    declareParameter(source + "." + "obstacle_min_range", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "raytrace_max_range", rclcpp::ParameterValue(3.0));
    declareParameter(source + "." + "raytrace_min_range", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "init_scan_angle", rclcpp::ParameterValue(true));
    declareParameter(source + "." + "scan_link_offset", rclcpp::ParameterValue(2.0));

    node->get_parameter(name_ + "." + source + "." + "topic", topic);
    node->get_parameter(name_ + "." + source + "." + "sensor_frame", sensor_frame);
    node->get_parameter(
      name_ + "." + source + "." + "observation_persistence",
      observation_keep_time);
    node->get_parameter(
      name_ + "." + source + "." + "expected_update_rate",
      expected_update_rate);
    node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
    node->get_parameter(name_ + "." + source + "." + "min_obstacle_height", min_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", max_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);
    node->get_parameter(name_ + "." + source + "." + "marking", marking);
    node->get_parameter(name_ + "." + source + "." + "clearing", clearing);

    if (!(data_type == "PointCloud2" || data_type == "LaserScan")) {
      RCLCPP_FATAL(
        logger_,
        "Only topics that use point cloud2s or laser scans are currently supported");
      throw std::runtime_error(
              "Only topics that use point cloud2s or laser scans are currently supported");
    }

    // get the obstacle range for the sensor
    double obstacle_max_range, obstacle_min_range;
    node->get_parameter(name_ + "." + source + "." + "obstacle_max_range", obstacle_max_range);
    node->get_parameter(name_ + "." + source + "." + "obstacle_min_range", obstacle_min_range);

    // get the raytrace ranges for the sensor
    double raytrace_max_range, raytrace_min_range;
    node->get_parameter(name_ + "." + source + "." + "raytrace_min_range", raytrace_min_range);
    node->get_parameter(name_ + "." + source + "." + "raytrace_max_range", raytrace_max_range);

    node->get_parameter(name_ + "." + source + "." + "init_scan_angle", init_scan_angle);
    node->get_parameter(name_ + "." + source + "." + "scan_link_offset", scan_link_offset_);

    RCLCPP_DEBUG(
      logger_,
      "Creating an observation buffer for source %s, topic %s, frame %s",
      source.c_str(), topic.c_str(),
      sensor_frame.c_str());

    // create an observation buffer
    observation_buffers_.push_back(
      std::shared_ptr<ObservationBuffer
      >(
        new ObservationBuffer(
          node, topic, observation_keep_time, expected_update_rate,
          min_obstacle_height,
          max_obstacle_height, obstacle_max_range, obstacle_min_range, raytrace_max_range,
          raytrace_min_range, *tf_,
          global_frame_,
          sensor_frame, tf2::durationFromSec(transform_tolerance))));

    // check if we'll add this buffer to our marking observation buffers
    if (marking) {
      marking_buffers_.push_back(observation_buffers_.back());
    }

    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing) {
      clearing_buffers_.push_back(observation_buffers_.back());
    }

    RCLCPP_DEBUG(
      logger_,
      "Created an observation buffer for source %s, topic %s, global frame: %s, "
      "expected update rate: %.2f, observation persistence: %.2f",
      source.c_str(), topic.c_str(),
      global_frame_.c_str(), expected_update_rate, observation_keep_time);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    // create a callback for the topic
    if (data_type == "LaserScan") {
      auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
          rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);
      sub->unsubscribe();

      auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *sub, *tf_, global_frame_, 50,
        node->get_node_logging_interface(),
        node->get_node_clock_interface(),
        tf2::durationFromSec(transform_tolerance));

      if (inf_is_valid) {
        filter->registerCallback(
          std::bind(
            &ObstacleLayer::laserScanValidInfCallback, this, std::placeholders::_1,
            observation_buffers_.back()));

      } else {
        filter->registerCallback(
          std::bind(
            &ObstacleLayer::laserScanCallback, this, std::placeholders::_1,
            observation_buffers_.back()));
      }

      observation_subscribers_.push_back(sub);

      observation_notifiers_.push_back(filter);
      observation_notifiers_.back()->setTolerance(rclcpp::Duration::from_seconds(0.05));

    } else {
      auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
          rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);
      sub->unsubscribe();

      if (inf_is_valid) {
        RCLCPP_WARN(
          logger_,
          "obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *sub, *tf_, global_frame_, 50,
        node->get_node_logging_interface(),
        node->get_node_clock_interface(),
        tf2::durationFromSec(transform_tolerance));

      filter->registerCallback(
        std::bind(
          &ObstacleLayer::pointCloud2Callback, this, std::placeholders::_1,
          observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    if (sensor_frame != "") {
      std::vector<std::string> target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
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
    cost_translation_table_[0] = 0;  // NO obstacle
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

rcl_interfaces::msg::SetParametersResult
ObstacleLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "min_obstacle_height") {
        min_obstacle_height_ = parameter.as_double();
      } else if (param_name == name_ + "." + "max_obstacle_height") {
        max_obstacle_height_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      } else if (param_name == name_ + "." + "footprint_clearing_enabled") {
        footprint_clearing_enabled_ = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "combination_method") {
        combination_method_ = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

void
ObstacleLayer::initializeScanAngle(sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
  // [sungkyu.kang] initialize scan angles
  RCLCPP_INFO(logger_, "ObstacleLayer::initializeScanAngle");
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
ObstacleLayer::laserScanCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr message,
  const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer)
{
  if (!is_init_scan_angle_) {
    initializeScanAngle(message);
  }
  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message->header;

  // project the scan into a point cloud
  try {
    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
      global_frame_.c_str(),
      ex.what());
    projector_.projectLaser(*message, cloud);
  } catch (std::runtime_error & ex) {
    RCLCPP_WARN(
      logger_,
      "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
      " Ignore this message. what(): %s",
      ex.what());
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void
ObstacleLayer::laserScanValidInfCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr raw_message,
  const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer)
{
  if (!is_init_scan_angle_) {
    initializeScanAngle(raw_message);
  }
  // Filter positive infinities ("Inf"s) to max_range.
  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::msg::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++) {
    float range = message.ranges[i];
    if (!std::isfinite(range) && range > 0) {
      message.ranges[i] = message.range_max - epsilon;
    }
  }

  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  try {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
      global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  } catch (std::runtime_error & ex) {
    RCLCPP_WARN(
      logger_,
      "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
      " Ignore this message. what(): %s",
      ex.what());
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void
ObstacleLayer::pointCloud2Callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
  const std::shared_ptr<ObservationBuffer> & buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void 
ObstacleLayer::updateRobotYaw(double robot_yaw)
{
  // [sungkyu.kang] set current_robot_yaw_
  current_robot_yaw_ = robot_yaw;
  // RCLCPP_INFO(logger_, "updateBounds - robot_yaw: %f, current_robot_yaw_: %f", robot_yaw, current_robot_yaw_);
}

void
ObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  updateRobotYaw(robot_yaw);
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;

  // raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin();
    it != observations.end(); ++it)
  {
    const Observation & obs = *it;

    const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

    double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    // RCLCPP_INFO(logger_, "[ObstacleLayer] observation --------------------------- ");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      double px = *iter_x, py = *iter_y, pz = *iter_z;

      // if the obstacle is too low, we won't add it
      if (pz < min_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too low");
        continue;
      }

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too high");
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist =
        (px -
        obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y) +
        (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_max_range) {
        RCLCPP_DEBUG(logger_, "The point is too far away");
        continue;
      }

      // if the point is too close, do not conisder it
      if (sq_dist < sq_obstacle_min_range) {
        RCLCPP_DEBUG(logger_, "The point is too close");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my)) {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }

      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;

      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

}

void
ObstacleLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void
ObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  // if not current due to reset, set current now after clearing
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }

  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  switch (combination_method_) {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }

  // [sungkyu.kang] publish current master grid
  if (obstacle_grid_pub_->get_subscription_count() > 0) {
    prepareGrid(master_grid);
    obstacle_grid_pub_->publish(std::move(grid_));
  }

}

void ObstacleLayer::prepareGrid(nav2_costmap_2d::Costmap2D costmap)
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
  //   "\nObstacleLayer::prepareGrid %f, %f, %f.\n    %f, %f, %f, %f", current_robot_yaw_, current_robot_yaw_ - scan_start_angle_, current_robot_yaw_ - scan_end_angle_, normalLeftX, normalLeftY, normalRightX, normalRightY);
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
  // RCLCPP_INFO(logger_, "ObstacleLayer::prepareGrid finish ");
}


void
ObstacleLayer::addStaticObservation(
  nav2_costmap_2d::Observation & obs,
  bool marking, bool clearing)
{
  if (marking) {
    static_marking_observations_.push_back(obs);
  }
  if (clearing) {
    static_clearing_observations_.push_back(obs);
  }
}

void
ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking) {
    static_marking_observations_.clear();
  }
  if (clearing) {
    static_clearing_observations_.clear();
  }
}

bool
ObstacleLayer::getMarkingObservations(std::vector<Observation> & marking_observations) const
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i) {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  marking_observations.insert(
    marking_observations.end(),
    static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool
ObstacleLayer::getClearingObservations(std::vector<Observation> & clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i) {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  clearing_observations.insert(
    clearing_observations.end(),
    static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void
ObstacleLayer::raytraceFreespace(
  const Observation & clearing_observation, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  const sensor_msgs::msg::PointCloud2 & cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0)) {
    RCLCPP_WARN(
      logger_,
      "Sensor origin at (%.2f, %.2f) is out of map bounds (%.2f, %.2f) to (%.2f, %.2f). "
      "The costmap cannot raytrace for it.",
      ox, oy,
      origin_x_, origin_y_,
      origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY());
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;


  touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin
  // and clear obstacles along it
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    double wx = *iter_x;
    double wy = *iter_y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    if (wx < origin_x) {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y) {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1)) {
      continue;
    }

    unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
    unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_max_range, cell_raytrace_min_range);

    updateRaytraceBounds(
      ox, oy, wx, wy, clearing_observation.raytrace_max_range_,
      clearing_observation.raytrace_min_range_, min_x, min_y, max_x,
      max_y);
  }
}

void
ObstacleLayer::activate()
{
  for (auto & notifier : observation_notifiers_) {
    notifier->clear();
  }

  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->subscribe();
    }
  }
  resetBuffersLastUpdated();
}

void
ObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->unsubscribe();
    }
  }
}

void
ObstacleLayer::updateRaytraceBounds(
  double ox, double oy, double wx, double wy, double max_range, double min_range,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  if (full_distance < min_range) {
    return;
  }
  double scale = std::min(1.0, max_range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void
ObstacleLayer::reset()
{
  resetMaps();
  resetBuffersLastUpdated();
  current_ = false;
  was_reset_ = true;
}

void
ObstacleLayer::resetBuffersLastUpdated()
{
  for (unsigned int i = 0; i < observation_buffers_.size(); ++i) {
    if (observation_buffers_[i]) {
      observation_buffers_[i]->resetLastUpdated();
    }
  }
}

}  // namespace nav2_costmap_2d
