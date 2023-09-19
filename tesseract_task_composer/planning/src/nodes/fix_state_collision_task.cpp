/**
 * @file fix_state_collisions_task.cpp
 * @brief Task for process that pushes plan instructions to be out of collision
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <trajopt/problem_description.hpp>

#include <tesseract_environment/utils.h>

#include <tesseract_task_composer/planning/nodes/fix_state_collision_task.h>

#include <tesseract_command_language/utils.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_collision/core/serialization.h>

namespace tesseract_planning
{
bool stateInCollision(const Eigen::Ref<const Eigen::VectorXd>& start_pos,
                      const tesseract_common::ManipulatorInfo& manip_info,
                      const PlanningTaskComposerProblem& problem,
                      const FixStateCollisionProfile& profile,
                      tesseract_collision::ContactResultMap& contacts)
{
  using namespace tesseract_collision;
  using namespace tesseract_environment;

  tesseract_common::ManipulatorInfo mi = manip_info.getCombined(problem.manip_info);
  auto joint_group = problem.env->getJointGroup(mi.manipulator);

  DiscreteContactManager::Ptr manager = problem.env->getDiscreteContactManager();
  manager->setActiveCollisionObjects(joint_group->getActiveLinkNames());
  manager->applyContactManagerConfig(profile.collision_check_config.contact_manager_config);

  tesseract_common::TransformMap state = joint_group->calcFwdKin(start_pos);
  contacts.clear();
  checkTrajectoryState(contacts, *manager, state, profile.collision_check_config);
  if (contacts.empty())
  {
    CONSOLE_BRIDGE_logDebug("No collisions found");
    if (profile.collision_check_config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
      CONSOLE_BRIDGE_logDebug("StateInCollision does not support longest valid segment logic");
    return false;
  }

  CONSOLE_BRIDGE_logDebug("Waypoint is not contact free!");
  if (console_bridge::getLogLevel() < console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
  {
    for (const auto& pair : contacts)
    {
      for (const auto& contact : pair.second)
        CONSOLE_BRIDGE_logDebug(("Contact Results: Links: " + contact.link_names[0] + ", " + contact.link_names[1] +
                                 " Dist: " + std::to_string(contact.distance))
                                    .c_str());
    }
  }

  return true;
}

bool waypointInCollision(const WaypointPoly& waypoint,
                         const tesseract_common::ManipulatorInfo& manip_info,
                         const PlanningTaskComposerProblem& problem,
                         const FixStateCollisionProfile& profile,
                         tesseract_collision::ContactResultMap& contacts)
{
  if (waypoint.isCartesianWaypoint())
  {
    CONSOLE_BRIDGE_logDebug("WaypointInCollision, skipping cartesian waypoint!");
    return false;
  }

  // Get position associated with waypoint
  Eigen::VectorXd start_pos;
  try
  {
    start_pos = getJointPosition(waypoint);
  }
  catch (std::runtime_error& e)
  {
    CONSOLE_BRIDGE_logError("WaypointInCollision error: %s", e.what());
    return false;
  }

  return stateInCollision(start_pos, manip_info, problem, profile, contacts);
}

bool moveWaypointFromCollisionTrajopt(WaypointPoly& waypoint,
                                      const tesseract_common::ManipulatorInfo& manip_info,
                                      const PlanningTaskComposerProblem& problem,
                                      const FixStateCollisionProfile& profile)
{
  using namespace trajopt;

  if (waypoint.isCartesianWaypoint())
  {
    CONSOLE_BRIDGE_logDebug("MoveWaypointFromCollision, skipping cartesian waypoint!");
    return true;
  }

  // Get position associated with waypoint
  Eigen::VectorXd start_pos;
  try
  {
    start_pos = getJointPosition(waypoint);
  }
  catch (std::runtime_error& e)
  {
    CONSOLE_BRIDGE_logError("MoveWaypointFromCollision error: %s", e.what());
    return false;
  }
  auto num_jnts = static_cast<std::size_t>(start_pos.size());

  // Setup trajopt problem with basic info
  ProblemConstructionInfo pci(problem.env);
  pci.basic_info.n_steps = 1;
  pci.basic_info.manip = manip_info.getCombined(problem.manip_info).manipulator;
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.env->getJointGroup(pci.basic_info.manip);

  // Initialize trajectory to waypoint position
  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = tesseract_common::TrajArray(1, start_pos.size());
  pci.init_info.data.row(0) = start_pos.transpose();

  // Add constraint that position is allowed to be within a tolerance of the original position
  {
    Eigen::MatrixX2d limits = pci.kin->getLimits().joint_limits;
    Eigen::VectorXd range = limits.col(1).array() - limits.col(0).array();
    Eigen::VectorXd pos_tolerance = range * profile.jiggle_factor;
    Eigen::VectorXd neg_tolerance = range * -profile.jiggle_factor;

    auto jp = std::make_shared<JointPosTermInfo>();
    jp->coeffs = std::vector<double>(num_jnts, 1.0);
    jp->targets = std::vector<double>(start_pos.data(), start_pos.data() + start_pos.size());
    jp->upper_tols = std::vector<double>(pos_tolerance.data(), pos_tolerance.data() + pos_tolerance.size());
    jp->lower_tols = std::vector<double>(neg_tolerance.data(), neg_tolerance.data() + neg_tolerance.size());
    jp->first_step = 0;
    jp->last_step = 0;
    jp->name = "joint_pos";
    jp->term_type = TT_CNT;
    pci.cnt_infos.push_back(jp);
  }

  // Add a constraint that it must not be in collision
  {
    auto collision = std::make_shared<CollisionTermInfo>();
    collision->name = "collision";
    collision->term_type = TT_CNT;
    collision->evaluator_type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
    collision->first_step = 0;
    collision->last_step = 0;
    collision->info = trajopt_common::createSafetyMarginDataVector(
        pci.basic_info.n_steps,
        profile.collision_check_config.contact_manager_config.margin_data.getMaxCollisionMargin(),
        1);
    collision->use_weighted_sum = true;
    pci.cnt_infos.push_back(collision);
  }
  // Add an additional cost to collisions to help it converge
  {
    auto collision = std::make_shared<CollisionTermInfo>();
    collision->name = "collision";
    collision->term_type = TT_COST;
    collision->evaluator_type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
    collision->first_step = 0;
    collision->last_step = 0;
    collision->info = trajopt_common::createSafetyMarginDataVector(
        pci.basic_info.n_steps,
        profile.collision_check_config.contact_manager_config.margin_data.getMaxCollisionMargin(),
        20);
    collision->use_weighted_sum = true;
    pci.cost_infos.push_back(collision);
  }
  auto prob = ConstructProblem(pci);

  // Run trajopt optimization
  sco::BasicTrustRegionSQP opt(prob);
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();
  if (opt.results().status != sco::OptStatus::OPT_CONVERGED)
  {
    CONSOLE_BRIDGE_logError("MoveWaypointFromCollision did not converge");

    tesseract_collision::ContactResultMap collisions;
    tesseract_collision::DiscreteContactManager::Ptr manager = pci.env->getDiscreteContactManager();
    tesseract_common::TransformMap state = pci.kin->calcFwdKin(start_pos);
    manager->setActiveCollisionObjects(pci.kin->getActiveLinkNames());
    manager->applyContactManagerConfig(profile.collision_check_config.contact_manager_config);
    manager->setCollisionObjectsTransform(state);
    manager->contactTest(collisions, profile.collision_check_config.contact_request);

    for (const auto& collision : collisions)
    {
      std::stringstream ss;
      ss << "Discrete collision detected between '" << collision.first.first << "' and '" << collision.first.second
         << "' with distance " << collision.second.front().distance << std::endl;

      CONSOLE_BRIDGE_logError(ss.str().c_str());
    }

    return false;
  }
  Eigen::VectorXd results(start_pos.size());
  results = getTraj(opt.x(), prob->GetVars()).row(0);
  return setJointPosition(waypoint, results);
}

bool moveWaypointFromCollisionRandomSampler(WaypointPoly& waypoint,
                                            const tesseract_common::ManipulatorInfo& manip_info,
                                            const PlanningTaskComposerProblem& problem,
                                            const FixStateCollisionProfile& profile)
{
  if (waypoint.isCartesianWaypoint())
  {
    CONSOLE_BRIDGE_logDebug("MoveWaypointFromCollisionRandomSampler, skipping cartesian waypoint!");
    return true;
  }

  // Get position associated with waypoint
  Eigen::VectorXd start_pos;
  try
  {
    start_pos = getJointPosition(waypoint);
  }
  catch (std::runtime_error& e)
  {
    CONSOLE_BRIDGE_logError("MoveWaypointFromCollision error: %s", e.what());
    return false;
  }

  tesseract_common::ManipulatorInfo mi = manip_info.getCombined(problem.manip_info);
  tesseract_kinematics::JointGroup::UPtr kin = problem.env->getJointGroup(mi.manipulator);
  Eigen::MatrixXd limits = kin->getLimits().joint_limits;
  Eigen::VectorXd range = limits.col(1).array() - limits.col(0).array();

  assert(start_pos.size() == range.size());
  for (int i = 0; i < profile.sampling_attempts; i++)
  {
    Eigen::VectorXd start_sampled_pos =
        start_pos + Eigen::VectorXd::Random(start_pos.size()).cwiseProduct(range) * profile.jiggle_factor;

    // Make sure it doesn't violate joint limits
    Eigen::VectorXd sampled_pos = start_sampled_pos;
    sampled_pos = sampled_pos.cwiseMax(limits.col(0));
    sampled_pos = sampled_pos.cwiseMin(limits.col(1));

    tesseract_collision::ContactResultMap contacts;
    if (!stateInCollision(sampled_pos, manip_info, problem, profile, contacts))
    {
      return setJointPosition(waypoint, sampled_pos);
    }
  }

  return false;
}

bool applyCorrectionWorkflow(WaypointPoly& waypoint,
                             const tesseract_common::ManipulatorInfo& manip_info,
                             const PlanningTaskComposerProblem& problem,
                             const FixStateCollisionProfile& profile,
                             tesseract_collision::ContactResultMap& contacts)
{
  for (const auto& method : profile.correction_workflow)
  {
    switch (method)  // NOLINT
    {
      case FixStateCollisionProfile::CorrectionMethod::NONE:
        return false;  // No correction and in collision, so return false
      case FixStateCollisionProfile::CorrectionMethod::TRAJOPT:
        if (moveWaypointFromCollisionTrajopt(waypoint, manip_info, problem, profile))
          return true;
        break;
      case FixStateCollisionProfile::CorrectionMethod::RANDOM_SAMPLER:
        if (moveWaypointFromCollisionRandomSampler(waypoint, manip_info, problem, profile))
          return true;
        break;
    }
  }
  // If all methods have tried without returning, then correction failed
  waypointInCollision(waypoint, manip_info, problem, profile, contacts);  // NOLINT Not sure why clang-tidy errors here
  return false;
}

FixStateCollisionTask::FixStateCollisionTask() : TaskComposerTask("FixStateCollisionTask", true) {}
FixStateCollisionTask::FixStateCollisionTask(std::string name,
                                             std::string input_key,
                                             std::string output_key,
                                             bool is_conditional)
  : TaskComposerTask(std::move(name), is_conditional)
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

FixStateCollisionTask::FixStateCollisionTask(std::string name,
                                             const YAML::Node& config,
                                             const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  if (input_keys_.empty())
    throw std::runtime_error("FixStateCollisionTask, config missing 'inputs' entry");

  if (input_keys_.size() > 1)
    throw std::runtime_error("FixStateCollisionTask, config 'inputs' entry currently only supports one input key");

  if (output_keys_.empty())
    throw std::runtime_error("FixStateCollisionTask, config missing 'outputs' entry");

  if (output_keys_.size() > 1)
    throw std::runtime_error("FixStateCollisionTask, config 'outputs' entry currently only supports one output key");
}

TaskComposerNodeInfo::UPtr FixStateCollisionTask::runImpl(TaskComposerContext& context,
                                                          OptionalTaskComposerExecutor /*executor*/) const
{
  // Get the problem
  auto& problem = dynamic_cast<PlanningTaskComposerProblem&>(*context.problem);

  auto info = std::make_unique<FixStateCollisionTaskInfo>(*this);
  info->return_value = 0;
  info->env = problem.env;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = context.data_storage->getData(input_keys_[0]);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input to FixStateCollision must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  auto& ci = input_data_poly.as<CompositeInstruction>();

  // Get Composite Profile
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, problem.composite_profile_remapping);
  auto cur_composite_profile = getProfile<FixStateCollisionProfile>(
      name_, profile, *problem.profiles, std::make_shared<FixStateCollisionProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  switch (cur_composite_profile->mode)
  {
    case FixStateCollisionProfile::Settings::START_ONLY:
    {
      MoveInstructionPoly* first_mi = ci.getFirstMoveInstruction();
      if (first_mi != nullptr)
      {
        info->contact_results.resize(1);
        tesseract_common::ManipulatorInfo mi = ci.getManipulatorInfo().getCombined(first_mi->getManipulatorInfo());
        if (waypointInCollision(first_mi->getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[0]))
        {
          CONSOLE_BRIDGE_logInform("FixStateCollisionTask is modifying the input instructions");
          if (!applyCorrectionWorkflow(
                  first_mi->getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[0]))
          {
            // If the output key is not the same as the input key the output data should be assigned the input data for
            // error branching
            if (output_keys_[0] != input_keys_[0])
              context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

            // Save space
            for (auto& contact_map : info->contact_results)
              contact_map.shrinkToFit();

            info->message = "Failed to correct state in collision";
            return info;
          }
        }
      }
    }
    break;
    case FixStateCollisionProfile::Settings::END_ONLY:
    {
      MoveInstructionPoly* last_mi = ci.getLastMoveInstruction();
      if (last_mi != nullptr)
      {
        info->contact_results.resize(1);
        tesseract_common::ManipulatorInfo mi = ci.getManipulatorInfo().getCombined(last_mi->getManipulatorInfo());
        if (waypointInCollision(last_mi->getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[0]))
        {
          CONSOLE_BRIDGE_logInform("FixStateCollisionTask is modifying the input instructions");
          if (!applyCorrectionWorkflow(
                  last_mi->getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[0]))
          {
            // If the output key is not the same as the input key the output data should be assigned the input data for
            // error branching
            if (output_keys_[0] != input_keys_[0])
              context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

            // Save space
            for (auto& contact_map : info->contact_results)
              contact_map.shrinkToFit();

            info->message = "Failed to correct state in collision";
            return info;
          }
        }
      }
    }
    break;
    case FixStateCollisionProfile::Settings::INTERMEDIATE_ONLY:
    {
      auto flattened = ci.flatten(moveFilter);
      info->contact_results.resize(flattened.size());
      if (flattened.empty())
      {
        if (output_keys_[0] != input_keys_[0])
          context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

        info->message = "FixStateCollisionTask found no MoveInstructions to process";
        info->return_value = 1;
        CONSOLE_BRIDGE_logWarn("%s", info->message.c_str());
        return info;
      }

      if (flattened.size() <= 2)
      {
        if (output_keys_[0] != input_keys_[0])
          context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

        info->message = "FixStateCollisionTask found no intermediate MoveInstructions to process";
        info->return_value = 1;
        CONSOLE_BRIDGE_logWarn("%s", info->message.c_str());
        return info;
      }

      bool in_collision = false;
      std::vector<bool> in_collision_vec(flattened.size());
      for (std::size_t i = 1; i < flattened.size() - 1; i++)
      {
        auto& plan = flattened[i].get().as<MoveInstructionPoly>();
        tesseract_common::ManipulatorInfo mi = ci.getManipulatorInfo().getCombined(plan.getManipulatorInfo());
        in_collision_vec[i] =
            waypointInCollision(plan.getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[i]);
        in_collision |= in_collision_vec[i];
      }
      if (!in_collision)
        break;

      CONSOLE_BRIDGE_logInform("FixStateCollisionTask is modifying the input instructions");
      for (std::size_t i = 1; i < flattened.size() - 1; i++)
      {
        if (in_collision_vec[i])
        {
          auto& plan = flattened[i].get().as<MoveInstructionPoly>();
          tesseract_common::ManipulatorInfo mi = ci.getManipulatorInfo().getCombined(plan.getManipulatorInfo());
          if (!applyCorrectionWorkflow(
                  plan.getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[i]))
          {
            // If the output key is not the same as the input key the output data should be assigned the input data for
            // error branching
            if (output_keys_[0] != input_keys_[0])
              context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

            // Save space
            for (auto& contact_map : info->contact_results)
              contact_map.shrinkToFit();

            info->message = "Failed to correct state in collision";
            return info;
          }
        }
      }
    }
    break;
    case FixStateCollisionProfile::Settings::ALL:
    {
      auto flattened = ci.flatten(moveFilter);
      info->contact_results.resize(flattened.size());
      if (flattened.empty())
      {
        if (output_keys_[0] != input_keys_[0])
          context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

        info->message = "FixStateCollisionTask found no MoveInstructions to process";
        info->return_value = 1;
        CONSOLE_BRIDGE_logWarn("%s", info->message.c_str());
        return info;
      }

      bool in_collision = false;
      std::vector<bool> in_collision_vec(flattened.size());
      for (std::size_t i = 0; i < flattened.size(); i++)
      {
        auto& plan = flattened[i].get().as<MoveInstructionPoly>();
        tesseract_common::ManipulatorInfo mi = ci.getManipulatorInfo().getCombined(plan.getManipulatorInfo());
        in_collision_vec[i] =
            waypointInCollision(plan.getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[i]);
        in_collision |= in_collision_vec[i];
      }
      if (!in_collision)
        break;

      CONSOLE_BRIDGE_logInform("FixStateCollisionTask is modifying the input instructions");
      for (std::size_t i = 0; i < flattened.size(); i++)
      {
        if (in_collision_vec[i])
        {
          auto& plan = flattened[i].get().as<MoveInstructionPoly>();
          tesseract_common::ManipulatorInfo mi = ci.getManipulatorInfo().getCombined(plan.getManipulatorInfo());
          if (!applyCorrectionWorkflow(
                  plan.getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[i]))
          {
            // If the output key is not the same as the input key the output data should be assigned the input data for
            // error branching
            if (output_keys_[0] != input_keys_[0])
              context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

            // Save space
            for (auto& contact_map : info->contact_results)
              contact_map.shrinkToFit();

            info->message = "Failed to correct state in collision";
            return info;
          }
        }
      }
    }
    break;
    case FixStateCollisionProfile::Settings::ALL_EXCEPT_START:
    {
      auto flattened = ci.flatten(moveFilter);
      info->contact_results.resize(flattened.size());
      if (flattened.empty())
      {
        if (output_keys_[0] != input_keys_[0])
          context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

        info->message = "FixStateCollisionTask found no MoveInstructions to process";
        info->return_value = 1;
        CONSOLE_BRIDGE_logWarn("%s", info->message.c_str());
        return info;
      }

      bool in_collision = false;
      std::vector<bool> in_collision_vec(flattened.size());
      for (std::size_t i = 1; i < flattened.size(); i++)
      {
        auto& plan = flattened[i].get().as<MoveInstructionPoly>();
        tesseract_common::ManipulatorInfo mi = ci.getManipulatorInfo().getCombined(plan.getManipulatorInfo());
        in_collision_vec[i] =
            waypointInCollision(plan.getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[i]);
        in_collision |= in_collision_vec[i];
      }
      if (!in_collision)
        break;

      CONSOLE_BRIDGE_logInform("FixStateCollisionTask is modifying the input instructions");
      for (std::size_t i = 1; i < flattened.size(); i++)
      {
        if (in_collision_vec[i])
        {
          auto& plan = flattened[i].get().as<MoveInstructionPoly>();
          tesseract_common::ManipulatorInfo mi = ci.getManipulatorInfo().getCombined(plan.getManipulatorInfo());
          if (!applyCorrectionWorkflow(
                  plan.getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[i]))
          {
            // If the output key is not the same as the input key the output data should be assigned the input data for
            // error branching
            if (output_keys_[0] != input_keys_[0])
              context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

            // Save space
            for (auto& contact_map : info->contact_results)
              contact_map.shrinkToFit();

            info->message = "Failed to correct state in collision";
            return info;
          }
        }
      }
    }
    break;
    case FixStateCollisionProfile::Settings::ALL_EXCEPT_END:
    {
      auto flattened = ci.flatten(moveFilter);
      info->contact_results.resize(flattened.size());
      if (flattened.size() <= 1)
      {
        if (output_keys_[0] != input_keys_[0])
          context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

        info->message = "FixStateCollisionTask found no MoveInstructions to process";
        info->return_value = 1;
        CONSOLE_BRIDGE_logWarn("%s", info->message.c_str());
        return info;
      }

      bool in_collision = false;
      std::vector<bool> in_collision_vec(flattened.size());
      for (std::size_t i = 0; i < flattened.size() - 1; i++)
      {
        auto& plan = flattened[i].get().as<MoveInstructionPoly>();
        tesseract_common::ManipulatorInfo mi = ci.getManipulatorInfo().getCombined(plan.getManipulatorInfo());
        in_collision_vec[i] =
            waypointInCollision(plan.getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[i]);
        in_collision |= in_collision_vec[i];
      }
      if (!in_collision)
        break;

      CONSOLE_BRIDGE_logInform("FixStateCollisionTask is modifying the input instructions");
      for (std::size_t i = 0; i < flattened.size() - 1; i++)
      {
        if (in_collision_vec[i])
        {
          auto& plan = flattened[i].get().as<MoveInstructionPoly>();
          tesseract_common::ManipulatorInfo mi = ci.getManipulatorInfo().getCombined(plan.getManipulatorInfo());
          if (!applyCorrectionWorkflow(
                  plan.getWaypoint(), mi, problem, *cur_composite_profile, info->contact_results[i]))
          {
            // If the output key is not the same as the input key the output data should be assigned the input data for
            // error branching
            if (output_keys_[0] != input_keys_[0])
              context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

            // Save space
            for (auto& contact_map : info->contact_results)
              contact_map.shrinkToFit();

            info->message = "Failed to correct state in collision";
            return info;
          }
        }
      }
    }
    break;
    case FixStateCollisionProfile::Settings::DISABLED:
    {
      if (output_keys_[0] != input_keys_[0])
        context.data_storage->setData(output_keys_[0], context.data_storage->getData(input_keys_[0]));

      info->message = "Successful, DISABLED";
      info->return_value = 1;
      return info;
    }
  }

  context.data_storage->setData(output_keys_[0], input_data_poly);

  info->color = "green";
  info->message = "Successful";
  info->return_value = 1;
  CONSOLE_BRIDGE_logDebug("FixStateCollisionTask succeeded");
  return info;
}

bool FixStateCollisionTask::operator==(const FixStateCollisionTask& rhs) const
{
  return (TaskComposerTask::operator==(rhs));
}
bool FixStateCollisionTask::operator!=(const FixStateCollisionTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void FixStateCollisionTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

FixStateCollisionTaskInfo::FixStateCollisionTaskInfo(const FixStateCollisionTask& task) : TaskComposerNodeInfo(task) {}

TaskComposerNodeInfo::UPtr FixStateCollisionTaskInfo::clone() const
{
  return std::make_unique<FixStateCollisionTaskInfo>(*this);
}

bool FixStateCollisionTaskInfo::operator==(const FixStateCollisionTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  equal &= tesseract_common::pointersEqual(env, rhs.env);
  //  equal &= contact_results == rhs.contact_results;
  return equal;
}
bool FixStateCollisionTaskInfo::operator!=(const FixStateCollisionTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void FixStateCollisionTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
  ar& BOOST_SERIALIZATION_NVP(env);
  ar& BOOST_SERIALIZATION_NVP(contact_results);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::FixStateCollisionTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::FixStateCollisionTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::FixStateCollisionTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::FixStateCollisionTaskInfo)
