/**
 * @file ompl_default_plan_profile.cpp
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
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
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
OMPLDefaultPlanProfile::OMPLDefaultPlanProfile()
  : planners({ std::make_shared<const RRTConnectConfigurator>(), std::make_shared<const RRTConnectConfigurator>() })
{
}

std::unique_ptr<OMPLProblem>
OMPLDefaultPlanProfile::create(const MoveInstructionPoly& start_instruction,
                               const MoveInstructionPoly& end_instruction,
                               const tesseract_common::ManipulatorInfo& composite_mi,
                               const std::shared_ptr<const tesseract_environment::Environment>& env,
                               int n_output_states,
                               int index) const
{
  tesseract_kinematics::JointGroup::Ptr manip;
  if (composite_mi.manipulator.empty())
    throw std::runtime_error("OMPL, manipulator is empty!");

  try
  {
    tesseract_kinematics::KinematicGroup::Ptr kin_group;
    std::string error_msg;
    if (composite_mi.manipulator_ik_solver.empty())
    {
      kin_group = env->getKinematicGroup(composite_mi.manipulator);
      error_msg = "Failed to find kinematic group for manipulator '" + composite_mi.manipulator + "'";
    }
    else
    {
      kin_group = env->getKinematicGroup(composite_mi.manipulator, composite_mi.manipulator_ik_solver);
      error_msg = "Failed to find kinematic group for manipulator '" + composite_mi.manipulator + "' with solver '" +
                  composite_mi.manipulator_ik_solver + "'";
    }

    if (kin_group == nullptr)
    {
      CONSOLE_BRIDGE_logError("%s", error_msg.c_str());
      throw std::runtime_error(error_msg);
    }

    manip = kin_group;
  }
  catch (...)
  {
    manip = env->getJointGroup(composite_mi.manipulator);
  }

  if (!manip)
    throw std::runtime_error("Failed to get joint/kinematic group: " + composite_mi.manipulator);

  std::vector<std::string> joint_names = manip->getJointNames();
  std::vector<std::string> active_link_names = manip->getActiveLinkNames();

  /** @todo Should check that the joint names match the order of the manipulator */
  auto problem = std::make_unique<OMPLProblem>();
  problem->env = env;
  problem->env_state = env->getState();
  problem->manip = manip;
  problem->contact_checker = env->getDiscreteContactManager();
  problem->contact_checker->setCollisionObjectsTransform(problem->env_state.link_transforms);
  problem->contact_checker->setActiveCollisionObjects(active_link_names);

  setup(*problem);
  problem->n_output_states = n_output_states;

  if (end_instruction.getWaypoint().isJointWaypoint() || end_instruction.getWaypoint().isStateWaypoint())
  {
    assert(checkJointPositionFormat(joint_names, end_instruction.getWaypoint()));
    const Eigen::VectorXd& cur_position = getJointPosition(end_instruction.getWaypoint());
    applyGoalStates(*problem, cur_position);

    if (start_instruction.getWaypoint().isJointWaypoint() || start_instruction.getWaypoint().isStateWaypoint())
    {
      assert(checkJointPositionFormat(joint_names, start_instruction.getWaypoint()));
      const Eigen::VectorXd& prev_position = getJointPosition(start_instruction.getWaypoint());
      applyStartStates(*problem, prev_position);
    }
    else if (start_instruction.getWaypoint().isCartesianWaypoint())
    {
      const auto& prev_wp = start_instruction.getWaypoint().as<CartesianWaypointPoly>();
      applyStartStates(*problem, prev_wp.getTransform(), start_instruction, composite_mi);
    }
    else
    {
      throw std::runtime_error("OMPLMotionPlanner: unknown waypoint type");
    }

    return problem;
  }

  if (end_instruction.getWaypoint().isCartesianWaypoint())
  {
    const auto& cur_wp = end_instruction.getWaypoint().as<CartesianWaypointPoly>();
    applyGoalStates(*problem, cur_wp.getTransform(), end_instruction, composite_mi);

    if (index == 1)
    {
      if (start_instruction.getWaypoint().isJointWaypoint() || start_instruction.getWaypoint().isStateWaypoint())
      {
        assert(checkJointPositionFormat(joint_names, start_instruction.getWaypoint()));
        const Eigen::VectorXd& prev_position = getJointPosition(start_instruction.getWaypoint());
        applyStartStates(*problem, prev_position);
      }
      else if (start_instruction.getWaypoint().isCartesianWaypoint())
      {
        const auto& prev_wp = start_instruction.getWaypoint().as<CartesianWaypointPoly>();
        applyStartStates(*problem, prev_wp.getTransform(), start_instruction, composite_mi);
      }
      else
      {
        throw std::runtime_error("OMPLMotionPlanner: unknown waypoint type");
      }
    }
    else
    {
      /** @todo Update. Extract the solution for the previous plan and set as the start */
      assert(false);
    }

    return problem;
  }

  throw std::runtime_error("OMPLMotionPlanner: unknown waypoint type");
}

void OMPLDefaultPlanProfile::setup(OMPLProblem& prob) const
{
  prob.planners = planners;
  prob.planning_time = planning_time;
  prob.max_solutions = max_solutions;
  prob.simplify = simplify;
  prob.optimize = optimize;
  prob.state_space = OMPLProblemStateSpace::REAL_STATE_SPACE;
  prob.contact_checker->applyContactManagerConfig(collision_check_config.contact_manager_config);

  std::vector<std::string> joint_names = prob.manip->getJointNames();
  auto dof = static_cast<unsigned>(prob.manip->numJoints());
  auto limits = prob.manip->getLimits().joint_limits;

  prob.extractor = [dof](const ompl::base::State* state) -> Eigen::Map<Eigen::VectorXd> {
    return tesseract_planning::RealVectorStateSpaceExtractor(state, dof);
  };

  // Construct the OMPL state space for this manipulator
  ompl::base::StateSpacePtr state_space_ptr;

  auto rss = std::make_shared<ompl::base::RealVectorStateSpace>();
  for (unsigned i = 0; i < dof; ++i)
    rss->addDimension(joint_names[i], limits(i, 0), limits(i, 1));

  rss->setStateSamplerAllocator(createStateSamplerAllocator(prob));

  state_space_ptr = rss;

  // Setup Longest Valid Segment
  processLongestValidSegment(state_space_ptr, collision_check_config);

  // Create Simple Setup from state space
  prob.simple_setup = std::make_shared<ompl::geometric::SimpleSetup>(state_space_ptr);

  // Setup state validators
  auto csvc = std::make_shared<CompoundStateValidator>();
  ompl::base::StateValidityCheckerPtr svc_without_collision = createStateValidator(prob);
  if (svc_without_collision != nullptr)
    csvc->addStateValidator(svc_without_collision);

  auto svc_collision = createCollisionStateValidator(prob);
  if (svc_collision != nullptr)
    csvc->addStateValidator(std::move(svc_collision));

  prob.simple_setup->setStateValidityChecker(csvc);

  // Setup motion validation (i.e. collision checking)
  auto mv = createMotionValidator(prob, svc_without_collision);
  if (mv != nullptr)
    prob.simple_setup->getSpaceInformation()->setMotionValidator(std::move(mv));

  // make sure the planners run until the time limit, and get the best possible solution
  if (prob.optimize)
  {
    auto obj = createOptimizationObjective(prob);
    if (obj != nullptr)
      prob.simple_setup->getProblemDefinition()->setOptimizationObjective(std::move(obj));
  }
}

void OMPLDefaultPlanProfile::applyGoalStates(OMPLProblem& prob,
                                             const Eigen::Isometry3d& cartesian_waypoint,
                                             const MoveInstructionPoly& parent_instruction,
                                             const tesseract_common::ManipulatorInfo& manip_info)
{
  if (prob.state_space != OMPLProblemStateSpace::REAL_STATE_SPACE)
    throw std::runtime_error("OMPL profile only supports real state space");

  const auto dof = prob.manip->numJoints();
  tesseract_common::KinematicLimits limits = prob.manip->getLimits();

  assert(!(manip_info.empty() && parent_instruction.getManipulatorInfo().empty()));
  tesseract_common::ManipulatorInfo mi = manip_info.getCombined(parent_instruction.getManipulatorInfo());

  if (mi.manipulator.empty())
    throw std::runtime_error("OMPL, manipulator is empty!");

  if (mi.tcp_frame.empty())
    throw std::runtime_error("OMPL, tcp_frame is empty!");

  if (mi.working_frame.empty())
    throw std::runtime_error("OMPL, working_frame is empty!");

  Eigen::Isometry3d tcp_offset = prob.env->findTCPOffset(mi);

  Eigen::Isometry3d tcp_frame_cwp = cartesian_waypoint * tcp_offset.inverse();

  /** @todo Need to add Descartes pose sample to ompl profile */
  tesseract_kinematics::KinGroupIKInput ik_input(tcp_frame_cwp, mi.working_frame, mi.tcp_frame);
  tesseract_kinematics::IKSolutions joint_solutions =
      std::dynamic_pointer_cast<const tesseract_kinematics::KinematicGroup>(prob.manip)
          ->calcInvKin({ ik_input }, Eigen::VectorXd::Zero(dof));
  auto goal_states = std::make_shared<ompl::base::GoalStates>(prob.simple_setup->getSpaceInformation());
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
      CONSOLE_BRIDGE_logDebug("In OMPLDefaultPlanProfile: Goal state has invalid bounds");
    }

    // Get discrete contact manager for testing provided start and end position
    // This is required because collision checking happens in motion validators now
    // instead of the isValid function to avoid unnecessary collision checks.
    if (!checkStateInCollision(prob, solution, contact_map_vec[i]))
    {
      {
        ompl::base::ScopedState<> goal_state(prob.simple_setup->getStateSpace());
        for (unsigned j = 0; j < dof; ++j)
          goal_state[j] = solution[static_cast<Eigen::Index>(j)];

        goal_states->addState(goal_state);
      }

      auto redundant_solutions = tesseract_kinematics::getRedundantSolutions<double>(
          solution, limits.joint_limits, prob.manip->getRedundancyCapableJointIndices());
      for (const auto& rs : redundant_solutions)
      {
        ompl::base::ScopedState<> goal_state(prob.simple_setup->getStateSpace());
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
    throw std::runtime_error("In OMPLDefaultPlanProfile: All goal states are either in collision or outside limits");
  }
  prob.simple_setup->setGoal(goal_states);
}

void OMPLDefaultPlanProfile::applyGoalStates(OMPLProblem& prob, const Eigen::VectorXd& joint_waypoint)
{
  if (prob.state_space != OMPLProblemStateSpace::REAL_STATE_SPACE)
    throw std::runtime_error("OMPL profile only supports real state space");

  const auto dof = prob.manip->numJoints();
  tesseract_common::KinematicLimits limits = prob.manip->getLimits();

  // Check limits
  Eigen::VectorXd solution = joint_waypoint;
  if (tesseract_common::satisfiesLimits<double>(solution, limits.joint_limits))
  {
    tesseract_common::enforceLimits<double>(solution, limits.joint_limits);
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("In OMPLDefaultPlanProfile: Goal state has invalid bounds");
  }

  // Get discrete contact manager for testing provided start and end position
  // This is required because collision checking happens in motion validators now
  // instead of the isValid function to avoid unnecessary collision checks.
  tesseract_collision::ContactResultMap contact_map;
  if (checkStateInCollision(prob, solution, contact_map))
  {
    CONSOLE_BRIDGE_logError("In OMPLDefaultPlanProfile: Goal state is in collision");
    for (const auto& contact_vec : contact_map)
      for (const auto& contact : contact_vec.second)
        CONSOLE_BRIDGE_logError(("Links: " + contact.link_names[0] + ", " + contact.link_names[1] +
                                 "  Distance: " + std::to_string(contact.distance))
                                    .c_str());
  }

  ompl::base::ScopedState<> goal_state(prob.simple_setup->getStateSpace());
  for (unsigned i = 0; i < dof; ++i)
    goal_state[i] = joint_waypoint[i];

  prob.simple_setup->setGoalState(goal_state);
}

void OMPLDefaultPlanProfile::applyStartStates(OMPLProblem& prob,
                                              const Eigen::Isometry3d& cartesian_waypoint,
                                              const MoveInstructionPoly& parent_instruction,
                                              const tesseract_common::ManipulatorInfo& manip_info)
{
  if (prob.state_space != OMPLProblemStateSpace::REAL_STATE_SPACE)
    throw std::runtime_error("OMPL profile only supports real state space");

  const auto dof = prob.manip->numJoints();
  tesseract_common::KinematicLimits limits = prob.manip->getLimits();

  assert(!(manip_info.empty() && parent_instruction.getManipulatorInfo().empty()));
  tesseract_common::ManipulatorInfo mi = manip_info.getCombined(parent_instruction.getManipulatorInfo());

  if (mi.manipulator.empty())
    throw std::runtime_error("OMPL, manipulator is empty!");

  if (mi.tcp_frame.empty())
    throw std::runtime_error("OMPL, tcp_frame is empty!");

  if (mi.working_frame.empty())
    throw std::runtime_error("OMPL, working_frame is empty!");

  Eigen::Isometry3d tcp = prob.env->findTCPOffset(mi);

  Eigen::Isometry3d tcp_frame_cwp = cartesian_waypoint * tcp.inverse();

  /** @todo Need to add Descartes pose sampler to ompl profile */
  /** @todo Need to also provide the seed instruction to use here */
  tesseract_kinematics::KinGroupIKInput ik_input(tcp_frame_cwp, mi.working_frame, mi.tcp_frame);
  tesseract_kinematics::IKSolutions joint_solutions =
      std::dynamic_pointer_cast<const tesseract_kinematics::KinematicGroup>(prob.manip)
          ->calcInvKin({ ik_input }, Eigen::VectorXd::Zero(dof));
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
      CONSOLE_BRIDGE_logDebug("In OMPLDefaultPlanProfile: Start state has invalid bounds");
    }

    // Get discrete contact manager for testing provided start and end position
    // This is required because collision checking happens in motion validators now
    // instead of the isValid function to avoid unnecessary collision checks.
    if (!checkStateInCollision(prob, solution, contact_map_vec[i]))
    {
      found_start_state = true;
      {
        ompl::base::ScopedState<> start_state(prob.simple_setup->getStateSpace());
        for (unsigned j = 0; j < dof; ++j)
          start_state[j] = solution[static_cast<Eigen::Index>(j)];

        prob.simple_setup->addStartState(start_state);
      }

      auto redundant_solutions = tesseract_kinematics::getRedundantSolutions<double>(
          solution, limits.joint_limits, prob.manip->getRedundancyCapableJointIndices());
      for (const auto& rs : redundant_solutions)
      {
        ompl::base::ScopedState<> start_state(prob.simple_setup->getStateSpace());
        for (unsigned j = 0; j < dof; ++j)
          start_state[j] = rs[static_cast<Eigen::Index>(j)];

        prob.simple_setup->addStartState(start_state);
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

void OMPLDefaultPlanProfile::applyStartStates(OMPLProblem& prob, const Eigen::VectorXd& joint_waypoint)
{
  if (prob.state_space != OMPLProblemStateSpace::REAL_STATE_SPACE)
    throw std::runtime_error("OMPL profile only supports real state space");

  const auto dof = prob.manip->numJoints();
  tesseract_common::KinematicLimits limits = prob.manip->getLimits();

  Eigen::VectorXd solution = joint_waypoint;

  if (tesseract_common::satisfiesLimits<double>(solution, limits.joint_limits))
  {
    tesseract_common::enforceLimits<double>(solution, limits.joint_limits);
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("In OMPLDefaultPlanProfile: Start state is outside limits");
  }

  // Get discrete contact manager for testing provided start and end position
  // This is required because collision checking happens in motion validators now
  // instead of the isValid function to avoid unnecessary collision checks.
  tesseract_collision::ContactResultMap contact_map;
  if (checkStateInCollision(prob, joint_waypoint, contact_map))
  {
    CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: Start state is in collision");
    for (const auto& contact_vec : contact_map)
      for (const auto& contact : contact_vec.second)
        CONSOLE_BRIDGE_logError(("Links: " + contact.link_names[0] + ", " + contact.link_names[1] +
                                 "  Distance: " + std::to_string(contact.distance))
                                    .c_str());
  }

  ompl::base::ScopedState<> start_state(prob.simple_setup->getStateSpace());
  for (unsigned i = 0; i < dof; ++i)
    start_state[i] = joint_waypoint[i];

  prob.simple_setup->addStartState(start_state);
}

std::function<ompl::base::StateSamplerPtr(const ompl::base::StateSpace*)>
OMPLDefaultPlanProfile::createStateSamplerAllocator(OMPLProblem& prob) const
{
  Eigen::MatrixX2d limits = prob.manip->getLimits().joint_limits;
  Eigen::VectorXd weights = Eigen::VectorXd::Ones(prob.manip->numJoints());
  return [weights, limits](const ompl::base::StateSpace* state_space) -> ompl::base::StateSamplerPtr {
    return allocWeightedRealVectorStateSampler(state_space, weights, limits);
  };
}

std::unique_ptr<ompl::base::StateValidityChecker>
OMPLDefaultPlanProfile::createStateValidator(OMPLProblem& /*prob*/) const
{
  return nullptr;
}

std::unique_ptr<ompl::base::StateValidityChecker>
OMPLDefaultPlanProfile::createCollisionStateValidator(OMPLProblem& prob) const
{
  if (collision_check_config.type == tesseract_collision::CollisionEvaluatorType::DISCRETE ||
      collision_check_config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    return std::make_unique<StateCollisionValidator>(
        prob.simple_setup->getSpaceInformation(), *prob.env, prob.manip, collision_check_config, prob.extractor);
  }

  return nullptr;
}

std::unique_ptr<ompl::base::MotionValidator>
OMPLDefaultPlanProfile::createMotionValidator(OMPLProblem& prob,
                                              const ompl::base::StateValidityCheckerPtr& svc_without_collision) const
{
  if (collision_check_config.type != tesseract_collision::CollisionEvaluatorType::NONE)
  {
    if (collision_check_config.type == tesseract_collision::CollisionEvaluatorType::CONTINUOUS ||
        collision_check_config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
    {
      return std::make_unique<ContinuousMotionValidator>(prob.simple_setup->getSpaceInformation(),
                                                         svc_without_collision,
                                                         *prob.env,
                                                         prob.manip,
                                                         collision_check_config,
                                                         prob.extractor);
    }

    // Collision checking is preformed using the state validator which this calls.
    return std::make_unique<DiscreteMotionValidator>(prob.simple_setup->getSpaceInformation());
  }

  return nullptr;
}

std::unique_ptr<ompl::base::OptimizationObjective>
OMPLDefaultPlanProfile::createOptimizationObjective(OMPLProblem& prob) const
{
  return std::make_unique<ompl::base::PathLengthOptimizationObjective>(prob.simple_setup->getSpaceInformation());
}

template <class Archive>
void OMPLDefaultPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLPlanProfile);
  ar& BOOST_SERIALIZATION_NVP(planning_time);
  ar& BOOST_SERIALIZATION_NVP(max_solutions);
  ar& BOOST_SERIALIZATION_NVP(simplify);
  ar& BOOST_SERIALIZATION_NVP(optimize);
  ar& BOOST_SERIALIZATION_NVP(planners);
  ar& BOOST_SERIALIZATION_NVP(collision_check_config);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::OMPLDefaultPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::OMPLDefaultPlanProfile)
