// Copyright 2018-2021 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__PID_LONGITUDINAL_CONTROLLER__LONGITUDINAL_CONTROLLER_UTILS_HPP_
#define AUTOWARE__PID_LONGITUDINAL_CONTROLLER__LONGITUDINAL_CONTROLLER_UTILS_HPP_

#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/interpolation/spherical_linear_interpolation.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "tf2/utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <experimental/optional>  // NOLINT

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace autoware::motion::control::pid_longitudinal_controller
{
namespace longitudinal_utils
{

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;

/**
 * @brief check if trajectory is invalid or not
 */
bool isValidTrajectory(const Trajectory & traj);

/**
 * @brief calculate distance to stopline from current vehicle position where velocity is 0
 */
double calcStopDistance(
  const Pose & current_pose, const Trajectory & traj, const double max_dist, const double max_yaw);

/**
 * @brief calculate pitch angle from estimated current pose
 */
double getPitchByPose(const Quaternion & quaternion);

/**
 * @brief calculate pitch angle from trajectory on map
 * NOTE: there is currently no z information so this always returns 0.0
 * @param [in] trajectory input trajectory
 * @param [in] start_idx nearest index to current vehicle position
 * @param [in] wheel_base length of wheel base
 */
double getPitchByTraj(
  const Trajectory & trajectory, const size_t start_idx, const double wheel_base);

/**
 * @brief calculate vehicle pose after time delay by moving the vehicle at current velocity and
 * acceleration for delayed time
 */
Pose calcPoseAfterTimeDelay(
  const Pose & current_pose, const double delay_time, const double current_vel,
  const double current_acc);

/**
 * @brief apply linear interpolation to trajectory point that is nearest to a certain point
 * @param [in] points trajectory points
 * @param [in] point Interpolated point is nearest to this point.
 */
template <class T>
std::pair<TrajectoryPoint, size_t> lerpTrajectoryPoint(
  const T & points, const Pose & pose, const double max_dist, const double max_yaw)
{
  TrajectoryPoint interpolated_point;
  const size_t seg_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, pose, max_dist, max_yaw);

  const double len_to_interpolated =
    autoware::motion_utils::calcLongitudinalOffsetToSegment(points, seg_idx, pose.position);
  const double len_segment =
    autoware::motion_utils::calcSignedArcLength(points, seg_idx, seg_idx + 1);
  const double interpolate_ratio = std::clamp(len_to_interpolated / len_segment, 0.0, 1.0);

  {
    const size_t i = seg_idx;

    interpolated_point.pose.position.x = autoware::interpolation::lerp(
      points.at(i).pose.position.x, points.at(i + 1).pose.position.x, interpolate_ratio);
    interpolated_point.pose.position.y = autoware::interpolation::lerp(
      points.at(i).pose.position.y, points.at(i + 1).pose.position.y, interpolate_ratio);
    interpolated_point.pose.position.z = autoware::interpolation::lerp(
      points.at(i).pose.position.z, points.at(i + 1).pose.position.z, interpolate_ratio);
    interpolated_point.pose.orientation = autoware::interpolation::lerpOrientation(
      points.at(i).pose.orientation, points.at(i + 1).pose.orientation, interpolate_ratio);
    interpolated_point.longitudinal_velocity_mps = autoware::interpolation::lerp(
      points.at(i).longitudinal_velocity_mps, points.at(i + 1).longitudinal_velocity_mps,
      interpolate_ratio);
    interpolated_point.lateral_velocity_mps = autoware::interpolation::lerp(
      points.at(i).lateral_velocity_mps, points.at(i + 1).lateral_velocity_mps, interpolate_ratio);
    interpolated_point.acceleration_mps2 = autoware::interpolation::lerp(
      points.at(i).acceleration_mps2, points.at(i + 1).acceleration_mps2, interpolate_ratio);
    interpolated_point.heading_rate_rps = autoware::interpolation::lerp(
      points.at(i).heading_rate_rps, points.at(i + 1).heading_rate_rps, interpolate_ratio);
  }

  return std::make_pair(interpolated_point, seg_idx);
}

/**
 * @brief limit variable whose differential is within a certain value
 * @param [in] input_val current value
 * @param [in] prev_val previous value
 * @param [in] dt time between current and previous one
 * @param [in] lim_val limitation value for differential
 */
double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val);

/**
 * @brief limit variable whose differential is within a certain value
 * @param [in] input_val current value
 * @param [in] prev_val previous value
 * @param [in] dt time between current and previous one
 * @param [in] max_val maximum value for differential
 * @param [in] min_val minimum value for differential
 */
double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val);

/**
 * @brief calculate the projected pose after distance from the current index
 * @param [in] src_idx current index
 * @param [in] distance distance to project
 * @param [in] trajectory reference trajectory
 */
geometry_msgs::msg::Pose findTrajectoryPoseAfterDistance(
  const size_t src_idx, const double distance,
  const autoware_planning_msgs::msg::Trajectory & trajectory);

}  // namespace longitudinal_utils
}  // namespace autoware::motion::control::pid_longitudinal_controller

#endif  // AUTOWARE__PID_LONGITUDINAL_CONTROLLER__LONGITUDINAL_CONTROLLER_UTILS_HPP_
