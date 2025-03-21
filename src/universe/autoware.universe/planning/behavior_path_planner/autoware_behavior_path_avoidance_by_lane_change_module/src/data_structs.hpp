// Copyright 2022 TIER IV, Inc.
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
#ifndef DATA_STRUCTS_HPP_
#define DATA_STRUCTS_HPP_

#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::AvoidanceParameters;

struct AvoidanceByLCParameters : public AvoidanceParameters
{
  // execute only when the target object longitudinal distance is larger than this param.
  double execute_object_longitudinal_margin{0.0};

  // execute only when lane change end point is before the object.
  bool execute_only_when_lane_change_finish_before_object{false};

  explicit AvoidanceByLCParameters(const AvoidanceParameters & param) : AvoidanceParameters(param)
  {
  }
};
}  // namespace autoware::behavior_path_planner

#endif  // DATA_STRUCTS_HPP_
