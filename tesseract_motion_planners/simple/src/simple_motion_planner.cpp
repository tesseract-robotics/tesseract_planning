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

#include <tesseract_motion_planners/simple/interpolation.h>
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

  // Place holder for start instruction
  MoveInstructionPoly null_instruction;

  // Process the instructions into the seed
  try
  {
    MoveInstructionPoly start_instruction_copy = null_instruction;
    MoveInstructionPoly start_instruction_seed_copy = null_instruction;
    seed =
        processCompositeInstruction(request.instructions, start_instruction_copy, start_instruction_seed_copy, request);
  }
  catch (std::exception& e)
  {
    CONSOLE_BRIDGE_logError("SimplePlanner failed to generate problem: %s.", e.what());
    response.successful = false;
    response.message = FAILED_TO_FIND_VALID_SOLUTION;
    return response;
  }

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
      seed.push_back(
          processCompositeInstruction(instruction.as<CompositeInstruction>(), prev_instruction, prev_seed, request));
    }
    else if (instruction.isMoveInstruction())
    {
      const auto& base_instruction = instruction.as<MoveInstructionPoly>();
      if (prev_instruction.isNull())
      {
        const std::string manipulator = request.instructions.getManipulatorInfo().manipulator;
        const std::string manipulator_ik_solver = request.instructions.getManipulatorInfo().manipulator_ik_solver;
        tesseract_kinematics::JointGroup::UPtr manip = request.env->getJointGroup(manipulator);

        prev_instruction = base_instruction;
        auto& start_waypoint = prev_instruction.getWaypoint();
        if (start_waypoint.isJointWaypoint() || start_waypoint.isStateWaypoint())
        {
          assert(checkJointPositionFormat(manip->getJointNames(), start_waypoint));
        }
        else if (start_waypoint.isCartesianWaypoint())
        {
          if (!start_waypoint.as<CartesianWaypointPoly>().hasSeed())
          {
            // Run IK to find solution closest to start
            KinematicGroupInstructionInfo info(prev_instruction, request, request.instructions.getManipulatorInfo());
            auto start_seed = getClosestJointSolution(info, request.env_state.getJointValues(manip->getJointNames()));
            start_waypoint.as<CartesianWaypointPoly>().setSeed(
                tesseract_common::JointState(manip->getJointNames(), start_seed));
          }
        }
        else
        {
          throw std::runtime_error("Unsupported waypoint type!");
        }
        prev_seed = prev_instruction;
        seed.push_back(prev_instruction);
        continue;
      }

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

      std::vector<MoveInstructionPoly> instruction_seed =
          plan_profile->generate(prev_instruction,
                                 prev_seed,
                                 base_instruction,
                                 next_instruction,
                                 request,
                                 request.instructions.getManipulatorInfo());

      // The data for the last instruction should be unchanged with exception to seed or tolerance joint state
      assert(instruction_seed.back().getMoveType() == base_instruction.getMoveType());
      assert(instruction_seed.back().getWaypoint().getType() == base_instruction.getWaypoint().getType());
      assert(instruction_seed.back().getPathProfile() == base_instruction.getPathProfile());
      assert(instruction_seed.back().getProfile() == base_instruction.getProfile());

      for (const auto& instr : instruction_seed)
        seed.push_back(instr);

      prev_instruction = base_instruction;
      prev_seed = instruction_seed.back();
    }
    else
    {
      seed.push_back(instruction);
    }
  }  // for (const auto& instruction : instructions)
  return seed;
}

}  // namespace tesseract_planning
