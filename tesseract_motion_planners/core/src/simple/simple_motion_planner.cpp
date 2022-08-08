/**
 * @file simple_motion_planner.cpp
 * @brief The simple planner is meant to be a tool for assigning values to the seed. The planner simply loops over all
 * of the MoveInstructions and then calls the appropriate function from the profile. These functions do not depend on
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
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

constexpr auto SOLUTION_FOUND{ "Found valid solution" };
constexpr auto ERROR_INVALID_INPUT{ "Input to planner is invalid. Check that instructions and seed are compatible" };
constexpr auto FAILED_TO_FIND_VALID_SOLUTION{ "Failed to find valid solution" };

namespace tesseract_planning
{
SimpleMotionPlanner::SimpleMotionPlanner(std::string name) : MotionPlanner(std::move(name)) {}

bool SimpleMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing planning is not implemented yet");
  return false;
}

void SimpleMotionPlanner::clear() {}

MotionPlanner::Ptr SimpleMotionPlanner::clone() const { return std::make_shared<SimpleMotionPlanner>(name_); }

PlannerResponse SimpleMotionPlanner::solve(const PlannerRequest& request) const
{
  PlannerResponse response;
  if (!checkRequest(request))
  {
    response.successful = false;
    response.message = ERROR_INVALID_INPUT;
    return response;
  }

  // Assume all the plan instructions have the same manipulator as the composite
  const std::string manipulator = request.instructions.getManipulatorInfo().manipulator;
  const std::string manipulator_ik_solver = request.instructions.getManipulatorInfo().manipulator_ik_solver;

  // Initialize
  tesseract_kinematics::JointGroup::UPtr manip = request.env->getJointGroup(manipulator);

  // Create seed
  CompositeInstruction seed;

  // Get the start waypoint/instruction
  MoveInstructionPoly start_instruction = getStartInstruction(request, request.env_state, *manip);

  // Set start instruction
  MoveInstructionPoly start_instruction_seed = start_instruction;
  start_instruction_seed.setMoveType(MoveInstructionType::START);

  // Process the instructions into the seed
  try
  {
    MoveInstructionPoly start_instruction_copy = start_instruction;
    MoveInstructionPoly start_instruction_seed_copy = start_instruction_seed;
    seed =
        processCompositeInstruction(request.instructions, start_instruction_copy, start_instruction_seed_copy, request);
  }
  catch (std::exception& e)
  {
    CONSOLE_BRIDGE_logError("SimplePlanner failed to generate problem: %s.", e.what());
    response.successful = false;
    response.message = ERROR_INVALID_INPUT;
    return response;
  }

  // Set seed start state
  seed.setStartInstruction(start_instruction_seed);

  // Fill out the response
  response.results = seed;

  // Enforce limits
  const Eigen::MatrixX2d joint_limits = manip->getLimits().joint_limits;
  auto results_flattened = response.results.flatten(&moveFilter);
  for (auto& inst : results_flattened)
  {
    auto& mi = inst.get().as<MoveInstructionPoly>();
    if (mi.getWaypoint().isJointWaypoint() || mi.getWaypoint().isStateWaypoint())
    {
      Eigen::VectorXd jp = getJointPosition(mi.getWaypoint());
      assert(tesseract_common::satisfiesPositionLimits<double>(jp, joint_limits));
      tesseract_common::enforcePositionLimits<double>(jp, joint_limits);
      setJointPosition(mi.getWaypoint(), jp);
    }
    else if (mi.getWaypoint().isCartesianWaypoint())
    {
      Eigen::VectorXd& jp = mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position;
      assert(tesseract_common::satisfiesPositionLimits<double>(jp, joint_limits));
      tesseract_common::enforcePositionLimits<double>(jp, joint_limits);
    }
    else
      throw std::runtime_error("Unsupported waypoint type.");
  }

  // Return success
  response.successful = true;
  response.message = SOLUTION_FOUND;
  return response;
}

MoveInstructionPoly SimpleMotionPlanner::getStartInstruction(const PlannerRequest& request,
                                                             const tesseract_scene_graph::SceneState& current_state,
                                                             const tesseract_kinematics::JointGroup& manip)
{
  // Get start instruction
  MoveInstructionPoly start_instruction = request.instructions.getStartInstruction();
  assert(start_instruction.isStart());
  auto& start_waypoint = start_instruction.getWaypoint();
  if (start_waypoint.isJointWaypoint() || start_waypoint.isStateWaypoint())
  {
    assert(checkJointPositionFormat(manip.getJointNames(), start_waypoint));
    return start_instruction;
  }

  if (start_waypoint.isCartesianWaypoint())
  {
    /** @todo Update to run IK to find solution closest to start */
    start_waypoint.as<CartesianWaypointPoly>().setSeed(
        tesseract_common::JointState(manip.getJointNames(), current_state.getJointValues(manip.getJointNames())));
    return start_instruction;
  }

  throw std::runtime_error("Unsupported waypoint type!");
}

CompositeInstruction SimpleMotionPlanner::processCompositeInstruction(const CompositeInstruction& instructions,
                                                                      MoveInstructionPoly& prev_instruction,
                                                                      MoveInstructionPoly& prev_seed,
                                                                      const PlannerRequest& request) const
{
  CompositeInstruction seed(instructions);
  seed.clear();

  for (std::size_t i = 0; i < instructions.size(); ++i)
  {
    const auto& instruction = instructions[i];

    if (instruction.isCompositeInstruction())
    {
      seed.appendInstruction(
          processCompositeInstruction(instruction.as<CompositeInstruction>(), prev_instruction, prev_seed, request));
    }
    else if (instruction.isMoveInstruction())
    {
      const auto& base_instruction = instruction.as<MoveInstructionPoly>();

      // Get the next plan instruction if it exists
      InstructionPoly next_instruction;
      for (std::size_t n = i + 1; n < instructions.size(); ++n)
      {
        if (instructions[n].isMoveInstruction())
        {
          next_instruction = instructions[n];
          break;
        }
      }

      // If a path profile exists for the instruction it should use that instead of the termination profile
      SimplePlannerPlanProfile::ConstPtr plan_profile;
      if (base_instruction.getPathProfile().empty())
      {
        std::string profile = getProfileString(name_, base_instruction.getProfile(), request.plan_profile_remapping);
        plan_profile = getProfile<SimplePlannerPlanProfile>(
            name_, profile, *request.profiles, std::make_shared<SimplePlannerLVSNoIKPlanProfile>());
        plan_profile = applyProfileOverrides(name_, profile, plan_profile, base_instruction.getProfileOverrides());
      }
      else
      {
        std::string profile =
            getProfileString(name_, base_instruction.getPathProfile(), request.plan_profile_remapping);
        plan_profile = getProfile<SimplePlannerPlanProfile>(
            name_, profile, *request.profiles, std::make_shared<SimplePlannerLVSNoIKPlanProfile>());
        plan_profile = applyProfileOverrides(name_, profile, plan_profile, base_instruction.getProfileOverrides());
      }

      if (!plan_profile)
        throw std::runtime_error("SimpleMotionPlanner: Invalid profile");

      CompositeInstruction instruction_seed = plan_profile->generate(prev_instruction,
                                                                     prev_seed,
                                                                     base_instruction,
                                                                     next_instruction,
                                                                     request,
                                                                     request.instructions.getManipulatorInfo());

      for (const auto& instr : instruction_seed)
        seed.appendInstruction(instr);

      prev_instruction = base_instruction;
      prev_seed = instruction_seed.back().as<MoveInstructionPoly>();
    }
    else
    {
      seed.appendInstruction(instruction);
    }
  }  // for (const auto& instruction : instructions)
  return seed;
}

}  // namespace tesseract_planning
