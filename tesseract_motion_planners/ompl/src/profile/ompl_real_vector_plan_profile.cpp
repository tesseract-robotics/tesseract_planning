/**
 * @file ompl_real_vector_plan_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalStates.h>
#include <boost/algorithm/string.hpp>
#include <console_bridge/console.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_plan_profile.h>
#include <tesseract_motion_planners/ompl/utils.h>

#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/state_collision_validator.h>
#include <tesseract_motion_planners/ompl/compound_state_validator.h>

#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/serialization.h>
#include <tesseract_environment/environment.h>

namespace tesseract_planning
{
OMPLRealVectorPlanProfile::OMPLRealVectorPlanProfile()
{
  solver_config.planners = { std::make_shared<const RRTConnectConfigurator>(),
                             std::make_shared<const RRTConnectConfigurator>() };
}

std::unique_ptr<OMPLSolverConfig> OMPLRealVectorPlanProfile::createSolverConfig() const
{
  return std::make_unique<OMPLSolverConfig>(solver_config);
}

OMPLStateExtractor OMPLRealVectorPlanProfile::createStateExtractor(const tesseract_kinematics::JointGroup& manip) const
{
  const auto dof = static_cast<unsigned>(manip.numJoints());
  return [dof](const ompl::base::State* state) -> Eigen::Map<Eigen::VectorXd> {
    return tesseract_planning::RealVectorStateSpaceExtractor(state, dof);
  };
}

std::unique_ptr<ompl::geometric::SimpleSetup>
OMPLRealVectorPlanProfile::createSimpleSetup(const MoveInstructionPoly& start_instruction,
                                             const MoveInstructionPoly& end_instruction,
                                             const tesseract_common::ManipulatorInfo& composite_mi,
                                             const std::shared_ptr<const tesseract_environment::Environment>& env) const
{
  // Start and End Manipulator Information
  // These should have the same manipulator name but could have different ik sovler name
  tesseract_common::ManipulatorInfo start_mi = composite_mi.getCombined(start_instruction.getManipulatorInfo());
  tesseract_common::ManipulatorInfo end_mi = composite_mi.getCombined(end_instruction.getManipulatorInfo());

  // Get kinematics
  tesseract_kinematics::JointGroup::Ptr manip = env->getJointGroup(end_mi.manipulator);
  const auto dof = static_cast<unsigned>(manip->numJoints());
  const std::vector<std::string> joint_names = manip->getJointNames();
  const Eigen::MatrixX2d limits = manip->getLimits().joint_limits;

  // Construct the OMPL state space for this manipulator
  ompl::base::StateSpacePtr state_space_ptr;

  auto rss = std::make_shared<ompl::base::RealVectorStateSpace>();
  for (unsigned i = 0; i < dof; ++i)
    rss->addDimension(joint_names[i], limits(i, 0), limits(i, 1));

  rss->setStateSamplerAllocator(createStateSamplerAllocator(env, manip));

  state_space_ptr = rss;

  // Setup Longest Valid Segment
  processLongestValidSegment(state_space_ptr, collision_check_config);

  // Create Simple Setup from state space
  auto simple_setup = std::make_unique<ompl::geometric::SimpleSetup>(state_space_ptr);

  // Create state extractor
  OMPLStateExtractor state_extractor = createStateExtractor(*manip);

  // Setup state validators
  auto csvc = std::make_shared<CompoundStateValidator>();
  ompl::base::StateValidityCheckerPtr svc_without_collision =
      createStateValidator(*simple_setup, env, manip, state_extractor);
  if (svc_without_collision != nullptr)
    csvc->addStateValidator(svc_without_collision);

  auto svc_collision = createCollisionStateValidator(*simple_setup, env, manip, state_extractor);
  if (svc_collision != nullptr)
    csvc->addStateValidator(std::move(svc_collision));

  simple_setup->setStateValidityChecker(csvc);

  // Setup motion validation (i.e. collision checking)
  auto mv = createMotionValidator(*simple_setup, env, manip, state_extractor, svc_without_collision);
  if (mv != nullptr)
    simple_setup->getSpaceInformation()->setMotionValidator(std::move(mv));

  // make sure the planners run until the time limit, and get the best possible solution
  if (solver_config.optimize)
  {
    auto obj = createOptimizationObjective(*simple_setup, env, manip, state_extractor);
    if (obj != nullptr)
      simple_setup->getProblemDefinition()->setOptimizationObjective(std::move(obj));
  }

  // Collision checker for validating start and goal states
  tesseract_collision::DiscreteContactManager::UPtr contact_checker = env->getDiscreteContactManager();
  contact_checker->applyContactManagerConfig(collision_check_config.contact_manager_config);
  contact_checker->setCollisionObjectsTransform(env->getState().link_transforms);

  // Add start states
  if (start_instruction.getWaypoint().isJointWaypoint() || start_instruction.getWaypoint().isStateWaypoint())
  {
    tesseract_kinematics::JointGroup::UPtr joint_group = env->getJointGroup(start_mi.manipulator);
    assert(checkJointPositionFormat(joint_group->getJointNames(), start_instruction.getWaypoint()));
    contact_checker->setActiveCollisionObjects(joint_group->getActiveLinkNames());
    const Eigen::VectorXd& cur_position = getJointPosition(start_instruction.getWaypoint());
    applyStartStates(*simple_setup, cur_position, *joint_group, *contact_checker);
  }
  else if (start_instruction.getWaypoint().isCartesianWaypoint())
  {
    const auto& cur_wp = start_instruction.getWaypoint().as<CartesianWaypointPoly>();
    Eigen::Isometry3d tcp_offset = env->findTCPOffset(start_mi);
    Eigen::Isometry3d tcp_frame_cwp = cur_wp.getTransform() * tcp_offset.inverse();
    tesseract_kinematics::KinematicGroup::UPtr kin_group;
    if (start_mi.manipulator_ik_solver.empty())
      kin_group = env->getKinematicGroup(start_mi.manipulator);
    else
      kin_group = env->getKinematicGroup(start_mi.manipulator, start_mi.manipulator_ik_solver);

    contact_checker->setActiveCollisionObjects(kin_group->getActiveLinkNames());
    tesseract_kinematics::KinGroupIKInput ik_input(tcp_frame_cwp, start_mi.working_frame, start_mi.tcp_frame);
    applyStartStates(*simple_setup, ik_input, *kin_group, *contact_checker);
  }
  else
  {
    throw std::runtime_error("OMPL, unsupported waypoint type");
  }

  // Add Goal states
  if (end_instruction.getWaypoint().isJointWaypoint() || end_instruction.getWaypoint().isStateWaypoint())
  {
    tesseract_kinematics::JointGroup::UPtr joint_group = env->getJointGroup(end_mi.manipulator);
    assert(checkJointPositionFormat(joint_group->getJointNames(), end_instruction.getWaypoint()));
    contact_checker->setActiveCollisionObjects(joint_group->getActiveLinkNames());
    const Eigen::VectorXd& cur_position = getJointPosition(end_instruction.getWaypoint());
    applyGoalStates(*simple_setup, cur_position, *joint_group, *contact_checker);
  }
  else if (end_instruction.getWaypoint().isCartesianWaypoint())
  {
    const auto& cur_wp = end_instruction.getWaypoint().as<CartesianWaypointPoly>();
    Eigen::Isometry3d tcp_offset = env->findTCPOffset(end_mi);
    Eigen::Isometry3d tcp_frame_cwp = cur_wp.getTransform() * tcp_offset.inverse();
    tesseract_kinematics::KinematicGroup::UPtr kin_group;
    if (end_mi.manipulator_ik_solver.empty())
      kin_group = env->getKinematicGroup(end_mi.manipulator);
    else
      kin_group = env->getKinematicGroup(end_mi.manipulator, end_mi.manipulator_ik_solver);

    contact_checker->setActiveCollisionObjects(kin_group->getActiveLinkNames());
    tesseract_kinematics::KinGroupIKInput ik_input(tcp_frame_cwp, end_mi.working_frame, end_mi.tcp_frame);
    applyGoalStates(*simple_setup, ik_input, *kin_group, *contact_checker);
  }
  else
  {
    throw std::runtime_error("OMPL, unsupported waypoint type");
  }

  return simple_setup;
}

void OMPLRealVectorPlanProfile::applyGoalStates(ompl::geometric::SimpleSetup& simple_setup,
                                                const tesseract_kinematics::KinGroupIKInput& ik_input,
                                                const tesseract_kinematics::KinematicGroup& manip,
                                                tesseract_collision::DiscreteContactManager& contact_checker)
{
  /** @todo Need to add Descartes pose sample to ompl profile */
  const auto dof = manip.numJoints();
  tesseract_common::KinematicLimits limits = manip.getLimits();
  tesseract_kinematics::IKSolutions joint_solutions = manip.calcInvKin({ ik_input }, Eigen::VectorXd::Zero(dof));
  auto goal_states = std::make_shared<ompl::base::GoalStates>(simple_setup.getSpaceInformation());
  std::vector<tesseract_collision::ContactResultMap> contact_map_vec(static_cast<std::size_t>(joint_solutions.size()));

  for (std::size_t i = 0; i < joint_solutions.size(); ++i)
  {
    Eigen::VectorXd& solution = joint_solutions[i];

    // Check limits
    if (tesseract_common::satisfiesLimits<double>(solution, limits.joint_limits))
    {
      tesseract_common::enforceLimits<double>(solution, limits.joint_limits);
    }
    else
    {
      CONSOLE_BRIDGE_logDebug("In OMPLRealVectorPlanProfile: Goal state has invalid bounds");
    }

    // Get discrete contact manager for testing provided start and end position
    // This is required because collision checking happens in motion validators now
    // instead of the isValid function to avoid unnecessary collision checks.
    if (!checkStateInCollision(contact_map_vec[i], contact_checker, manip, solution))
    {
      {
        ompl::base::ScopedState<> goal_state(simple_setup.getStateSpace());
        for (unsigned j = 0; j < dof; ++j)
          goal_state[j] = solution[static_cast<Eigen::Index>(j)];

        goal_states->addState(goal_state);
      }

      auto redundant_solutions = tesseract_kinematics::getRedundantSolutions<double>(
          solution, limits.joint_limits, manip.getRedundancyCapableJointIndices());
      for (const auto& rs : redundant_solutions)
      {
        ompl::base::ScopedState<> goal_state(simple_setup.getStateSpace());
        for (unsigned j = 0; j < dof; ++j)
          goal_state[j] = rs[static_cast<Eigen::Index>(j)];

        goal_states->addState(goal_state);
      }
    }
  }

  if (!goal_states->hasStates())
  {
    for (std::size_t i = 0; i < contact_map_vec.size(); i++)
      for (const auto& contact_vec : contact_map_vec[i])
        for (const auto& contact : contact_vec.second)
          CONSOLE_BRIDGE_logError(("Solution: " + std::to_string(i) + "  Links: " + contact.link_names[0] + ", " +
                                   contact.link_names[1] + "  Distance: " + std::to_string(contact.distance))
                                      .c_str());
    throw std::runtime_error("In OMPLRealVectorPlanProfile: All goal states are either in collision or outside limits");
  }
  simple_setup.setGoal(goal_states);
}

void OMPLRealVectorPlanProfile::applyGoalStates(ompl::geometric::SimpleSetup& simple_setup,
                                                const Eigen::VectorXd& joint_waypoint,
                                                const tesseract_kinematics::JointGroup& manip,
                                                tesseract_collision::DiscreteContactManager& contact_checker)
{
  const auto dof = manip.numJoints();
  tesseract_common::KinematicLimits limits = manip.getLimits();

  // Check limits
  Eigen::VectorXd solution = joint_waypoint;
  if (tesseract_common::satisfiesLimits<double>(solution, limits.joint_limits))
  {
    tesseract_common::enforceLimits<double>(solution, limits.joint_limits);
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("In OMPLRealVectorPlanProfile: Goal state has invalid bounds");
  }

  // Get discrete contact manager for testing provided start and end position
  // This is required because collision checking happens in motion validators now
  // instead of the isValid function to avoid unnecessary collision checks.
  tesseract_collision::ContactResultMap contact_map;
  if (checkStateInCollision(contact_map, contact_checker, manip, solution))
  {
    CONSOLE_BRIDGE_logError("In OMPLRealVectorPlanProfile: Goal state is in collision");
    for (const auto& contact_vec : contact_map)
      for (const auto& contact : contact_vec.second)
        CONSOLE_BRIDGE_logError(("Links: " + contact.link_names[0] + ", " + contact.link_names[1] +
                                 "  Distance: " + std::to_string(contact.distance))
                                    .c_str());
  }

  ompl::base::ScopedState<> goal_state(simple_setup.getStateSpace());
  for (unsigned i = 0; i < dof; ++i)
    goal_state[i] = joint_waypoint[i];

  simple_setup.setGoalState(goal_state);
}

void OMPLRealVectorPlanProfile::applyStartStates(ompl::geometric::SimpleSetup& simple_setup,
                                                 const tesseract_kinematics::KinGroupIKInput& ik_input,
                                                 const tesseract_kinematics::KinematicGroup& manip,
                                                 tesseract_collision::DiscreteContactManager& contact_checker)
{
  /** @todo Need to add Descartes pose sampler to ompl profile */
  /** @todo Need to also provide the seed instruction to use here */
  const auto dof = manip.numJoints();
  tesseract_common::KinematicLimits limits = manip.getLimits();
  tesseract_kinematics::IKSolutions joint_solutions = manip.calcInvKin({ ik_input }, Eigen::VectorXd::Zero(dof));
  bool found_start_state = false;
  std::vector<tesseract_collision::ContactResultMap> contact_map_vec(joint_solutions.size());

  for (std::size_t i = 0; i < joint_solutions.size(); ++i)
  {
    Eigen::VectorXd& solution = joint_solutions[i];

    // Check limits
    if (tesseract_common::satisfiesLimits<double>(solution, limits.joint_limits))
    {
      tesseract_common::enforceLimits<double>(solution, limits.joint_limits);
    }
    else
    {
      CONSOLE_BRIDGE_logDebug("In OMPLRealVectorPlanProfile: Start state has invalid bounds");
    }

    // Get discrete contact manager for testing provided start and end position
    // This is required because collision checking happens in motion validators now
    // instead of the isValid function to avoid unnecessary collision checks.
    if (!checkStateInCollision(contact_map_vec[i], contact_checker, manip, solution))
    {
      found_start_state = true;
      {
        ompl::base::ScopedState<> start_state(simple_setup.getStateSpace());
        for (unsigned j = 0; j < dof; ++j)
          start_state[j] = solution[static_cast<Eigen::Index>(j)];

        simple_setup.addStartState(start_state);
      }

      auto redundant_solutions = tesseract_kinematics::getRedundantSolutions<double>(
          solution, limits.joint_limits, manip.getRedundancyCapableJointIndices());
      for (const auto& rs : redundant_solutions)
      {
        ompl::base::ScopedState<> start_state(simple_setup.getStateSpace());
        for (unsigned j = 0; j < dof; ++j)
          start_state[j] = rs[static_cast<Eigen::Index>(j)];

        simple_setup.addStartState(start_state);
      }
    }
  }

  if (!found_start_state)
  {
    for (std::size_t i = 0; i < contact_map_vec.size(); i++)
      for (const auto& contact_vec : contact_map_vec[i])
        for (const auto& contact : contact_vec.second)
          CONSOLE_BRIDGE_logError(("Solution: " + std::to_string(i) + "  Links: " + contact.link_names[0] + ", " +
                                   contact.link_names[1] + "  Distance: " + std::to_string(contact.distance))
                                      .c_str());
    throw std::runtime_error("In OMPLPlannerFreespaceConfig: All start states are either in collision or outside "
                             "limits");
  }
}

void OMPLRealVectorPlanProfile::applyStartStates(ompl::geometric::SimpleSetup& simple_setup,
                                                 const Eigen::VectorXd& joint_waypoint,
                                                 const tesseract_kinematics::JointGroup& manip,
                                                 tesseract_collision::DiscreteContactManager& contact_checker)
{
  const auto dof = manip.numJoints();
  tesseract_common::KinematicLimits limits = manip.getLimits();

  Eigen::VectorXd solution = joint_waypoint;
  if (tesseract_common::satisfiesLimits<double>(solution, limits.joint_limits))
  {
    tesseract_common::enforceLimits<double>(solution, limits.joint_limits);
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("In OMPLRealVectorPlanProfile: Start state is outside limits");
  }

  // Get discrete contact manager for testing provided start and end position
  // This is required because collision checking happens in motion validators now
  // instead of the isValid function to avoid unnecessary collision checks.
  tesseract_collision::ContactResultMap contact_map;
  if (checkStateInCollision(contact_map, contact_checker, manip, solution))
  {
    CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: Start state is in collision");
    for (const auto& contact_vec : contact_map)
      for (const auto& contact : contact_vec.second)
        CONSOLE_BRIDGE_logError(("Links: " + contact.link_names[0] + ", " + contact.link_names[1] +
                                 "  Distance: " + std::to_string(contact.distance))
                                    .c_str());
  }

  ompl::base::ScopedState<> start_state(simple_setup.getStateSpace());
  for (unsigned i = 0; i < dof; ++i)
    start_state[i] = joint_waypoint[i];

  simple_setup.addStartState(start_state);
}

std::function<std::shared_ptr<ompl::base::StateSampler>(const ompl::base::StateSpace*)>
OMPLRealVectorPlanProfile::createStateSamplerAllocator(
    const std::shared_ptr<const tesseract_environment::Environment>& /*env*/,
    const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip) const
{
  Eigen::MatrixX2d limits = manip->getLimits().joint_limits;
  Eigen::VectorXd weights = Eigen::VectorXd::Ones(manip->numJoints());
  return [weights, limits](const ompl::base::StateSpace* state_space) -> ompl::base::StateSamplerPtr {
    return allocWeightedRealVectorStateSampler(state_space, weights, limits);
  };
}

std::unique_ptr<ompl::base::StateValidityChecker> OMPLRealVectorPlanProfile::createStateValidator(
    const ompl::geometric::SimpleSetup& /*simple_setup*/,
    const std::shared_ptr<const tesseract_environment::Environment>& /*env*/,
    const std::shared_ptr<const tesseract_kinematics::JointGroup>& /*manip*/,
    const OMPLStateExtractor& /*state_extractor*/) const
{
  return nullptr;
}

std::unique_ptr<ompl::base::StateValidityChecker> OMPLRealVectorPlanProfile::createCollisionStateValidator(
    const ompl::geometric::SimpleSetup& simple_setup,
    const std::shared_ptr<const tesseract_environment::Environment>& env,
    const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
    const OMPLStateExtractor& state_extractor) const
{
  if (collision_check_config.type == tesseract_collision::CollisionEvaluatorType::DISCRETE ||
      collision_check_config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    return std::make_unique<StateCollisionValidator>(
        simple_setup.getSpaceInformation(), *env, manip, collision_check_config, state_extractor);
  }

  return nullptr;
}

std::unique_ptr<ompl::base::MotionValidator> OMPLRealVectorPlanProfile::createMotionValidator(
    const ompl::geometric::SimpleSetup& simple_setup,
    const std::shared_ptr<const tesseract_environment::Environment>& env,
    const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
    const OMPLStateExtractor& state_extractor,
    const std::shared_ptr<ompl::base::StateValidityChecker>& svc_without_collision) const
{
  if (collision_check_config.type != tesseract_collision::CollisionEvaluatorType::NONE)
  {
    if (collision_check_config.type == tesseract_collision::CollisionEvaluatorType::CONTINUOUS ||
        collision_check_config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
    {
      return std::make_unique<ContinuousMotionValidator>(simple_setup.getSpaceInformation(),
                                                         svc_without_collision,
                                                         *env,
                                                         manip,
                                                         collision_check_config,
                                                         state_extractor);
    }

    // Collision checking is preformed using the state validator which this calls.
    return std::make_unique<DiscreteMotionValidator>(simple_setup.getSpaceInformation());
  }

  return nullptr;
}

std::unique_ptr<ompl::base::OptimizationObjective> OMPLRealVectorPlanProfile::createOptimizationObjective(
    const ompl::geometric::SimpleSetup& simple_setup,
    const std::shared_ptr<const tesseract_environment::Environment>& /*env*/,
    const std::shared_ptr<const tesseract_kinematics::JointGroup>& /*manip*/,
    const OMPLStateExtractor& /*state_extractor*/) const
{
  return std::make_unique<ompl::base::PathLengthOptimizationObjective>(simple_setup.getSpaceInformation());
}

template <class Archive>
void OMPLRealVectorPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlanProfile);
  ar& BOOST_SERIALIZATION_NVP(solver_config);
  ar& BOOST_SERIALIZATION_NVP(collision_check_config);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::OMPLRealVectorPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::OMPLRealVectorPlanProfile)
