/**
 * @file TrajOptIfopt_default_move_profile.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_MOVE_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_MOVE_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_waypoint_config.h>

namespace YAML
{
class Node;
}

namespace tesseract_planning
{
class TrajOptIfoptDefaultMoveProfile : public TrajOptIfoptMoveProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptDefaultMoveProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptDefaultMoveProfile>;

  TrajOptIfoptDefaultMoveProfile();
  TrajOptIfoptDefaultMoveProfile(const YAML::Node& config,
                                 const tesseract_common::ProfilePluginFactory& plugin_factory);

  TrajOptIfoptCartesianWaypointConfig cartesian_cost_config;
  TrajOptIfoptCartesianWaypointConfig cartesian_constraint_config;
  TrajOptIfoptJointWaypointConfig joint_cost_config;
  TrajOptIfoptJointWaypointConfig joint_constraint_config;

  TrajOptIfoptWaypointInfo create(const MoveInstructionPoly& move_instruction,
                                  const tesseract_common::ManipulatorInfo& composite_manip_info,
                                  const std::shared_ptr<const tesseract_environment::Environment>& env,
                                  int index) const override;

  bool operator==(const TrajOptIfoptDefaultMoveProfile& rhs) const;
  bool operator!=(const TrajOptIfoptDefaultMoveProfile& rhs) const;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TrajOptIfopt_IFOPT_DEFAULT_MOVE_PROFILE_H
