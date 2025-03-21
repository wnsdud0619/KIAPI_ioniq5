// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__MANAGER_HPP_

#include "autoware/behavior_path_goal_planner_module/goal_planner_parameters.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

class GoalPlannerModuleManager : public SceneModuleManagerInterface
{
public:
  static GoalPlannerParameters initGoalPlannerParameters(
    rclcpp::Node * node, const std::string & base_ns);

public:
  GoalPlannerModuleManager() : SceneModuleManagerInterface{"goal_planner"} {}

  void init(rclcpp::Node * node) override;

  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override;

  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) override;

  bool isSimultaneousExecutableAsApprovedModule() const override;

  bool isSimultaneousExecutableAsCandidateModule() const override;

private:
  std::shared_ptr<GoalPlannerParameters> parameters_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__MANAGER_HPP_
