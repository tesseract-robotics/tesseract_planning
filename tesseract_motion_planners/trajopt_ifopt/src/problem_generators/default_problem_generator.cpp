/**
 * @file default_problem_generator.cpp
 * @brief Generates a trajopt problem from a planner request
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_motion_planners/trajopt_ifopt/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>

#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/trajopt_ifopt.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
/// @todo: Restructure this into several smaller functions that are testable and easier to understand
std::shared_ptr<TrajOptIfoptProblem>
DefaultTrajOptIfoptProblemGenerator(const std::string& name,
                                    const PlannerRequest& request,
                                    const TrajOptIfoptPlanProfileMap& plan_profiles,
                                    const TrajOptIfoptCompositeProfileMap& composite_profiles)
{
  // Store fixed steps
  std::vector<int> fixed_steps;

  // Create the problem
  auto problem = std::make_shared<TrajOptIfoptProblem>();
  problem->environment = request.env;
  problem->env_state = request.env_state;
  problem->nlp = std::make_shared<trajopt_sqp::TrajOptQPProblem>();

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!request.instructions.getManipulatorInfo().empty());
  const ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();
  const std::string& manipulator = composite_mi.manipulator;
  auto kin = request.env->getManipulatorManager()->getFwdKinematicSolver(manipulator);
  auto inv_kin = request.env->getManipulatorManager()->getInvKinematicSolver(manipulator);
  assert(kin);
  problem->manip_fwd_kin = kin;
  problem->manip_inv_kin = inv_kin;

  // Apply Solver parameters
  std::string profile = request.instructions.getProfile();
  ProfileDictionary::ConstPtr profile_overrides = request.instructions.profile_overrides;
  profile = getProfileString(profile, name, PlannerProfileRemapping());
  //  TrajOptSolverProfile::ConstPtr solver_profile =
  //      getProfile<TrajOptSolverProfile>(profile, solver_profiles, std::make_shared<TrajOptDefaultSolverProfile>());
  //  solver_profile = applyProfileOverrides(name, solver_profile, profile_overrides);
  //  if (!solver_profile)
  //    throw std::runtime_error("TrajOptSolverConfig: Invalid profile");

  //  solver_profile->apply(*pci);

  // Flatten input instructions
  auto instructions_flat = flattenProgram(request.instructions);
  auto seed_flat_pattern = flattenProgramToPattern(request.seed, request.instructions);
  auto seed_flat = flattenProgram(request.seed);

  // Get kinematics information
  tesseract_environment::Environment::ConstPtr env = request.env;
  tesseract_environment::AdjacencyMap map(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);
  const std::vector<std::string>& active_links = map.getActiveLinkNames();

  // Check seed should have start instruction
  assert(request.seed.hasStartInstruction());

  // ----------------
  // Setup variables
  // ----------------
  // Create the vector of variables to be optimized
  Eigen::MatrixX2d joint_limits_eigen = kin->getLimits().joint_limits;

  // Setup start waypoint
  std::size_t start_index = 0;  // If it has a start instruction then skip first instruction in instructions_flat
  int index = 0;
  Waypoint start_waypoint{ NullWaypoint() };
  Instruction placeholder_instruction{ NullInstruction() };
  const Instruction* start_instruction = nullptr;
  if (request.instructions.hasStartInstruction())
  {
    assert(isPlanInstruction(request.instructions.getStartInstruction()));
    start_instruction = &(request.instructions.getStartInstruction());
    if (isPlanInstruction(*start_instruction))
    {
      const auto& temp = start_instruction->as<PlanInstruction>();
      assert(temp.isStart());
      start_waypoint = temp.getWaypoint();
      profile = temp.getProfile();
      profile_overrides = temp.profile_overrides;
    }
    else
    {
      throw std::runtime_error("DefaultTrajoptIfoptProblemGenerator: Unsupported start instruction type!");
    }

    // Create a variable for each instruction in the seed, setting to correct initial state
    for (std::size_t i = 0; i < seed_flat.size(); i++)
    {
      assert(isMoveInstruction(seed_flat[i]));
      auto var = std::make_shared<trajopt_ifopt::JointPosition>(
          getJointPosition(kin->getJointNames(), seed_flat[i].get().as<MoveInstruction>().getWaypoint()),
          kin->getJointNames(),
          "Joint_Position_" + std::to_string(i));
      var->SetBounds(joint_limits_eigen);
      problem->vars.push_back(var);
      problem->nlp->addVariableSet(var);
    }

    ++start_index;
  }
  else  // If not start instruction is given, take the current state
  {
    Eigen::VectorXd current_jv = request.env_state->getJointValues(kin->getJointNames());
    StateWaypoint swp(kin->getJointNames(), current_jv);

    MoveInstruction temp_move(swp, MoveInstructionType::START);
    placeholder_instruction = temp_move;
    start_instruction = &placeholder_instruction;
    start_waypoint = swp;

    // Create a variable for each instruction in the seed, setting to correct initial state
    auto var = std::make_shared<trajopt_ifopt::JointPosition>(getJointPosition(kin->getJointNames(), start_waypoint),
                                                              kin->getJointNames(),
                                                              "Joint_Position_" + std::to_string(0));
    var->SetBounds(joint_limits_eigen);
    problem->vars.push_back(var);
    problem->nlp->addVariableSet(var);

    for (std::size_t i = 0; i < seed_flat.size(); i++)
    {
      assert(isMoveInstruction(seed_flat[i]));
      auto var = std::make_shared<trajopt_ifopt::JointPosition>(
          getJointPosition(kin->getJointNames(), seed_flat[i].get().as<MoveInstruction>().getWaypoint()),
          kin->getJointNames(),
          "Joint_Position_" + std::to_string(i + 1));
      var->SetBounds(joint_limits_eigen);
      problem->vars.push_back(var);
      problem->nlp->addVariableSet(var);
    }
  }

  profile = getProfileString(profile, name, request.plan_profile_remapping);
  TrajOptIfoptPlanProfile::ConstPtr start_plan_profile =
      getProfile<TrajOptIfoptPlanProfile>(profile, plan_profiles, std::make_shared<TrajOptIfoptDefaultPlanProfile>());
  start_plan_profile = applyProfileOverrides(name, start_plan_profile, profile_overrides);
  if (!start_plan_profile)
    throw std::runtime_error("TrajOptIfoptPlannerUniversalConfig: Invalid profile");

  // Add start waypoint
  if (isCartesianWaypoint(start_waypoint))
  {
    const auto& cwp = start_waypoint.as<CartesianWaypoint>();
    start_plan_profile->apply(*problem, cwp.waypoint, *start_instruction, composite_mi, active_links, index);
  }
  else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
  {
    assert(checkJointPositionFormat(problem->manip_fwd_kin->getJointNames(), start_waypoint));
    bool toleranced = false;

    if (isJointWaypoint(start_waypoint))
    {
      toleranced = start_waypoint.as<JointWaypoint>().isToleranced();
      start_plan_profile->apply(
          *problem, start_waypoint.as<JointWaypoint>(), *start_instruction, composite_mi, active_links, index);
    }
    else if (isStateWaypoint(start_waypoint))
    {
      JointWaypoint jwp(start_waypoint.as<StateWaypoint>().joint_names, start_waypoint.as<StateWaypoint>().position);
      start_plan_profile->apply(*problem, jwp, *start_instruction, composite_mi, active_links, index);
    }
    else
      throw std::runtime_error("Unsupported start_waypoint type.");

    // Add to fixed indices
    if (!toleranced)
      fixed_steps.push_back(index);
  }
  else
  {
    throw std::runtime_error("TrajOpt Problem Generator: unknown waypoint type.");
  }

  ++index;

  // ----------------
  // Translate TCL for PlanInstructions
  // ----------------
  // Transform plan instructions into trajopt cost and constraints
  for (std::size_t i = start_index; i < instructions_flat.size(); i++)
  {
    const auto& instruction = instructions_flat[i].get();
    if (isPlanInstruction(instruction))
    {
      assert(isPlanInstruction(instruction));
      const auto& plan_instruction = instruction.as<PlanInstruction>();

      // If plan instruction has manipulator information then use it over the one provided by the composite.
      ManipulatorInfo manip_info = composite_mi.getCombined(plan_instruction.getManipulatorInfo());
      Eigen::Isometry3d tcp = request.env->findTCP(manip_info);

      assert(isCompositeInstruction(seed_flat_pattern[i].get()));
      const auto& seed_composite = seed_flat_pattern[i].get().as<tesseract_planning::CompositeInstruction>();
      auto interpolate_cnt = static_cast<int>(seed_composite.size());

      std::string profile = getProfileString(plan_instruction.getProfile(), name, request.plan_profile_remapping);
      TrajOptIfoptPlanProfile::ConstPtr cur_plan_profile = getProfile<TrajOptIfoptPlanProfile>(
          profile, plan_profiles, std::make_shared<TrajOptIfoptDefaultPlanProfile>());
      cur_plan_profile = applyProfileOverrides(name, cur_plan_profile, plan_instruction.profile_overrides);
      if (!cur_plan_profile)
        throw std::runtime_error("DefaultTrajoptIfoptProblemGenerator: Invalid profile");

      if (plan_instruction.isLinear())
      {
        if (isCartesianWaypoint(plan_instruction.getWaypoint()))
        {
          const auto& cur_wp = plan_instruction.getWaypoint().as<tesseract_planning::CartesianWaypoint>();

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = start_waypoint.as<CartesianWaypoint>().waypoint;
          }
          else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
          {
            assert(checkJointPositionFormat(kin->getJointNames(), start_waypoint));
            const Eigen::VectorXd& position = getJointPosition(start_waypoint);
            prev_pose = kin->calcFwdKin(position);
            prev_pose = env->getCurrentState()->link_transforms.at(kin->getBaseLinkName()) * prev_pose * tcp;
          }
          else
          {
            throw std::runtime_error("DefaultTrajoptIfoptProblemGenerator: unknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_wp, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            /** @todo Write a path constraint for this*/
            cur_plan_profile->apply(*problem, poses[p], plan_instruction, composite_mi, active_links, index);
            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*problem, cur_wp, plan_instruction, composite_mi, active_links, index);
          ++index;
        }
        else if (isJointWaypoint(plan_instruction.getWaypoint()) || isStateWaypoint(plan_instruction.getWaypoint()))
        {
          assert(checkJointPositionFormat(kin->getJointNames(), plan_instruction.getWaypoint()));
          bool toleranced = false;

          JointWaypoint cur_position;
          if (isJointWaypoint(plan_instruction.getWaypoint()))
          {
            toleranced = plan_instruction.getWaypoint().as<JointWaypoint>().isToleranced();
            cur_position = plan_instruction.getWaypoint().as<JointWaypoint>();
          }
          else if (isStateWaypoint(plan_instruction.getWaypoint()))
          {
            const auto& state_waypoint = plan_instruction.getWaypoint().as<StateWaypoint>();
            cur_position = JointWaypoint(state_waypoint.joint_names, state_waypoint.position);
          }
          else
            throw std::runtime_error("Unsupported waypoint type.");

          Eigen::Isometry3d cur_pose = kin->calcFwdKin(cur_position);
          cur_pose = env->getCurrentState()->link_transforms.at(kin->getBaseLinkName()) * cur_pose * tcp;

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = start_waypoint.as<CartesianWaypoint>().waypoint;
          }
          else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
          {
            assert(checkJointPositionFormat(problem->manip_fwd_kin->getJointNames(), start_waypoint));
            const Eigen::VectorXd& position = getJointPosition(start_waypoint);
            prev_pose = kin->calcFwdKin(position);
            prev_pose = env->getCurrentState()->link_transforms.at(kin->getBaseLinkName()) * prev_pose * tcp;
          }
          else
          {
            throw std::runtime_error("TrajOptPlannerUniversalConfig: uknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_pose, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            /** @todo Add path constraint for this */
            cur_plan_profile->apply(*problem, poses[p], plan_instruction, composite_mi, active_links, index);
            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*problem, cur_position, plan_instruction, composite_mi, active_links, index);

          // Add to fixed indices
          if (!toleranced)
            fixed_steps.push_back(index);

          ++index;
        }
        else
        {
          throw std::runtime_error("DefaultTrajoptIfoptProblemGenerator: unknown waypoint type");
        }
      }
      else if (plan_instruction.isFreespace())
      {
        if (isJointWaypoint(plan_instruction.getWaypoint()) || isStateWaypoint(plan_instruction.getWaypoint()))
        {
          assert(checkJointPositionFormat(kin->getJointNames(), plan_instruction.getWaypoint()));

          // Add intermediate points with path costs and constraints
          for (std::size_t s = 0; s < seed_composite.size() - 1; ++s)
            ++index;

          bool toleranced = false;

          // Add final point with waypoint costs and contraints
          JointWaypoint cur_position;
          if (isJointWaypoint(plan_instruction.getWaypoint()))
          {
            cur_position = plan_instruction.getWaypoint().as<JointWaypoint>();
            toleranced = cur_position.isToleranced();
          }
          else if (isStateWaypoint(plan_instruction.getWaypoint()))
          {
            const auto& state_waypoint = plan_instruction.getWaypoint().as<StateWaypoint>();
            cur_position = JointWaypoint(state_waypoint.joint_names, state_waypoint.position);
          }
          else
            throw std::runtime_error("Unsupported start_waypoint type.");

          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(*problem, cur_position, plan_instruction, manip_info, active_links, index);

          // Add to fixed indices
          if (!toleranced)
            fixed_steps.push_back(index);
        }
        else if (isCartesianWaypoint(plan_instruction.getWaypoint()))
        {
          const auto& cur_wp = plan_instruction.getWaypoint().as<CartesianWaypoint>();

          // Increment index to account for intermediate points in seed
          for (std::size_t s = 0; s < seed_composite.size() - 1; ++s)
            ++index;

          // Add final point with waypoint costs and contraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(*problem, cur_wp, plan_instruction, manip_info, active_links, index);
        }
        else
        {
          throw std::runtime_error("DefaultTrajoptIfoptProblemGenerator: unknown waypoint type");
        }

        ++index;
      }
      else
      {
        throw std::runtime_error("Unsupported!");
      }

      start_waypoint = plan_instruction.getWaypoint();
    }
  }

  // ----------------
  // Translate TCL for CompositeInstructions
  // ----------------
  profile = getProfileString(request.instructions.getProfile(), name, request.composite_profile_remapping);
  TrajOptIfoptCompositeProfile::ConstPtr cur_composite_profile = getProfile<TrajOptIfoptCompositeProfile>(
      profile, composite_profiles, std::make_shared<TrajOptIfoptDefaultCompositeProfile>());
  cur_composite_profile = applyProfileOverrides(name, cur_composite_profile, request.instructions.profile_overrides);
  if (!cur_composite_profile)
    throw std::runtime_error("DefaultTrajoptIfoptProblemGenerator: Invalid profile");

  cur_composite_profile->apply(
      *problem, 0, static_cast<int>(problem->vars.size()) - 1, composite_mi, active_links, fixed_steps);

  // ----------------
  // Return problem
  // ----------------
  problem->nlp->setup();
  return problem;
}

}  // namespace tesseract_planning
