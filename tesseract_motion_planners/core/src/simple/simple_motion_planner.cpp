/**
 * @file simple_motion_planner.cpp
 * @brief The simple planner is meant to be a tool for assigning values to the seed. The planner simply loops over all
 * of the PlanInstructions and then calls the appropriate function from the profile. These functions do not depend on
 * the seed, so this may be used to initialize the seed appropriately using e.g. linear interpolation.
 *
 * @author Matthew Powelson
 * @date July 23, 2020
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
#include <tesseract_environment/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
SimpleMotionPlannerStatusCategory::SimpleMotionPlannerStatusCategory(std::string name) : name_(std::move(name)) {}
const std::string& SimpleMotionPlannerStatusCategory::name() const noexcept { return name_; }
std::string SimpleMotionPlannerStatusCategory::message(int code) const
{
  switch (code)
  {
    case SolutionFound:
    {
      return "Found valid solution";
    }
    case ErrorInvalidInput:
    {
      return "Input to planner is invalid. Check that instructions and seed are compatible";
    }
    case FailedToFindValidSolution:
    {
      return "Failed to find valid solution";
    }
    default:
    {
      assert(false);
      return "";
    }
  }
}

SimpleMotionPlanner::SimpleMotionPlanner(std::string name)
  : name_(std::move(name)), status_category_(std::make_shared<const SimpleMotionPlannerStatusCategory>(name_))
{
  plan_profiles[DEFAULT_PROFILE_KEY] = std::make_shared<SimplePlannerLVSPlanProfile>();
}

const std::string& SimpleMotionPlanner::getName() const { return name_; }

bool SimpleMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing planning is not implemented yet");
  return false;
}

void SimpleMotionPlanner::clear() {}

MotionPlanner::Ptr SimpleMotionPlanner::clone() const { return std::make_shared<SimpleMotionPlanner>(name_); }

tesseract_common::StatusCode SimpleMotionPlanner::solve(const PlannerRequest& request,
                                                        PlannerResponse& response,
                                                        bool /*verbose*/) const
{
  if (!checkUserInput(request))
  {
    response.status =
        tesseract_common::StatusCode(SimpleMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }

  // Assume all the plan instructions have the same manipulator as the composite
  const std::string manipulator = request.instructions.getManipulatorInfo().manipulator;
  const std::string manipulator_ik_solver = request.instructions.getManipulatorInfo().manipulator_ik_solver;

  // Initialize
  tesseract_kinematics::JointGroup::UPtr manip = request.env->getJointGroup(manipulator);
  Waypoint start_waypoint{ NullWaypoint() };

  // Create seed
  CompositeInstruction seed;

  // Get the start waypoint/instruction
  PlanInstruction start_instruction = getStartInstruction(request, request.env_state, *manip);

  // Process the instructions into the seed
  try
  {
    PlanInstruction start_instruction_copy = start_instruction;
    seed = processCompositeInstruction(request.instructions, start_instruction_copy, request);
  }
  catch (std::exception& e)
  {
    CONSOLE_BRIDGE_logError("SimplePlanner failed to generate problem: %s.", e.what());
    response.status =
        tesseract_common::StatusCode(SimpleMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }

  // Set start instruction
  MoveInstruction move_start_instruction_seed(start_instruction.getWaypoint(),
                                              MoveInstructionType::START,
                                              start_instruction.getProfile(),
                                              start_instruction.getManipulatorInfo());
  move_start_instruction_seed.profile_overrides = start_instruction.profile_overrides;
  seed.setStartInstruction(move_start_instruction_seed);

  // Fill out the response
  response.results = seed;

  // Enforce limits
  auto results_flattened = flatten(response.results, &moveFilter);
  for (auto& inst : results_flattened)
  {
    auto& mi = inst.get().as<MoveInstruction>();
    Eigen::VectorXd jp = getJointPosition(mi.getWaypoint());
    assert(tesseract_common::satisfiesPositionLimits(jp, manip->getLimits().joint_limits));
    tesseract_common::enforcePositionLimits(jp, manip->getLimits().joint_limits);
    setJointPosition(mi.getWaypoint(), jp);
  }

  // Return success
  response.status = tesseract_common::StatusCode(SimpleMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
}

PlanInstruction SimpleMotionPlanner::getStartInstruction(const PlannerRequest& request,
                                                         const tesseract_scene_graph::SceneState& current_state,
                                                         const tesseract_kinematics::JointGroup& manip)
{
  // Create start instruction
  Waypoint start_waypoint{ NullWaypoint() };
  PlanInstruction start_instruction_seed(start_waypoint, PlanInstructionType::START);

  if (request.instructions.hasStartInstruction())
  {
    assert(isPlanInstruction(request.instructions.getStartInstruction()));
    const auto& start_instruction = request.instructions.getStartInstruction().as<PlanInstruction>();
    assert(start_instruction.isStart());
    start_waypoint = start_instruction.getWaypoint();

    if (isJointWaypoint(start_waypoint))
    {
      assert(checkJointPositionFormat(manip.getJointNames(), start_waypoint));
      const auto& jwp = start_waypoint.as<JointWaypoint>();
      start_instruction_seed.setWaypoint(StateWaypoint(jwp.joint_names, jwp.waypoint));
    }
    else if (isCartesianWaypoint(start_waypoint))
    {
      StateWaypoint temp(manip.getJointNames(), current_state.getJointValues(manip.getJointNames()));
      start_waypoint = temp;

      start_instruction_seed.setWaypoint(start_waypoint);
    }
    else if (isStateWaypoint(start_waypoint))
    {
      assert(checkJointPositionFormat(manip.getJointNames(), start_waypoint));
      start_instruction_seed.setWaypoint(start_waypoint);
    }
    else
    {
      throw std::runtime_error("Unsupported waypoint type!");
    }
    start_instruction_seed.setDescription(start_instruction.getDescription());
    start_instruction_seed.setProfile(start_instruction.getProfile());
    start_instruction_seed.profile_overrides = start_instruction.profile_overrides;
    start_instruction_seed.setManipulatorInfo(start_instruction.getManipulatorInfo());
  }
  else
  {
    StateWaypoint temp(manip.getJointNames(), current_state.getJointValues(manip.getJointNames()));
    start_waypoint = temp;

    start_instruction_seed.setWaypoint(start_waypoint);
  }

  return start_instruction_seed;
}

CompositeInstruction SimpleMotionPlanner::processCompositeInstruction(const CompositeInstruction& instructions,
                                                                      PlanInstruction& prev_instruction,
                                                                      const PlannerRequest& request) const
{
  CompositeInstruction seed(instructions);
  seed.clear();

  for (const auto& instruction : instructions)
  {
    if (isCompositeInstruction(instruction))
    {
      seed.push_back(processCompositeInstruction(instruction.as<CompositeInstruction>(), prev_instruction, request));
    }
    else if (isPlanInstruction(instruction))
    {
      const auto& base_instruction = instruction.as<PlanInstruction>();

      std::string profile = getProfileString(base_instruction.getProfile(), name_, request.plan_profile_remapping);
      SimplePlannerPlanProfile::ConstPtr start_plan_profile =
          getProfile<SimplePlannerPlanProfile>(profile, plan_profiles, std::make_shared<SimplePlannerLVSPlanProfile>());
      start_plan_profile = applyProfileOverrides(name_, start_plan_profile, base_instruction.profile_overrides);
      if (!start_plan_profile)
        throw std::runtime_error("SimpleMotionPlanner: Invalid start profile");

      seed.push_back(start_plan_profile->generate(
          prev_instruction, base_instruction, request, request.instructions.getManipulatorInfo()));

      prev_instruction = base_instruction;
    }
    else if (isMoveInstruction(instruction))
    {
      throw std::runtime_error("SimpleMotionPlanner: The input program includes MoveInstructions!");
    }
    else
    {
      seed.push_back(instruction);
    }
  }  // for (const auto& instruction : instructions)
  return seed;
}

bool SimpleMotionPlanner::checkUserInput(const PlannerRequest& request)
{
  // Check that parameters are valid
  if (request.env == nullptr)
  {
    CONSOLE_BRIDGE_logError("In SimpleMotionPlanner: env is a required parameter and has not been set");
    return false;
  }

  if (request.instructions.empty())
  {
    CONSOLE_BRIDGE_logError("SimpleMotionPlanner requires at least one instruction");
    return false;
  }

  return true;
}

}  // namespace tesseract_planning
