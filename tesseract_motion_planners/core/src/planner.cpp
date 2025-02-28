/**
 * @file planner.cpp
 * @brief Planner Interface Class.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/types.h>

#include <tesseract_common/joint_state.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>

namespace tesseract_planning
{
MotionPlanner::MotionPlanner(std::string name) : name_(std::move(name))
{
  if (name_.empty())
    throw std::runtime_error("MotionPlanner name is empty!");
}

const std::string& MotionPlanner::getName() const { return name_; }

bool MotionPlanner::checkRequest(const PlannerRequest& request)
{
  std::string reason;
  return checkRequest(request, reason);
}

bool MotionPlanner::checkRequest(const PlannerRequest& request, std::string& reason)
{
  // Check that parameters are valid
  if (request.env == nullptr)
  {
    reason = "PlannerRequest environment is nullptr";
    CONSOLE_BRIDGE_logError(reason.c_str());
    return false;
  }

  if (request.instructions.empty())
  {
    reason = "PlannerRequest instruction is empty";
    CONSOLE_BRIDGE_logError(reason.c_str());
    return false;
  }

  return true;
}

void MotionPlanner::assignSolution(MoveInstructionPoly& mi,
                                   const std::vector<std::string>& joint_names,
                                   const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                   bool format_result_as_input)
{
  if (format_result_as_input)
  {
    if (mi.getWaypoint().isStateWaypoint())
      return;

    if (mi.getWaypoint().isCartesianWaypoint())
    {
      auto& cwp = mi.getWaypoint().as<CartesianWaypointPoly>();
      cwp.setSeed(tesseract_common::JointState(joint_names, joint_values));
      return;
    }

    if (mi.getWaypoint().isJointWaypoint())
    {
      auto& jwp = mi.getWaypoint().as<JointWaypointPoly>();
      if (!jwp.isConstrained() || (jwp.isConstrained() && jwp.isToleranced()))
      {
        jwp.setNames(joint_names);
        jwp.setPosition(joint_values);
      }
      return;
    }

    throw std::runtime_error("Unsupported waypoint type!");
  }

  StateWaypointPoly swp = mi.createStateWaypoint();
  swp.setNames(joint_names);
  swp.setPosition(joint_values);
  mi.getWaypoint() = swp;
}

}  // namespace tesseract_planning
