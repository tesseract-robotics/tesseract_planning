#include <tesseract_motion_planners/ompl/profile/ompl_composite_profile_rvss.h>

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalStates.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/utils.h>
#include <tesseract_motion_planners/ompl/compound_state_validator.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/state_collision_validator.h>
#include <tesseract_motion_planners/ompl/utils.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>

namespace tesseract_planning
{
tesseract_collision::ContactResultMap checkCollision(const Eigen::VectorXd& state,
                                                     tesseract_kinematics::JointGroup::ConstPtr manip,
                                                     tesseract_collision::DiscreteContactManager::Ptr contact_checker)
{
  tesseract_common::TransformMap link_transforms = manip->calcFwdKin(state);

  for (const auto& link_name : contact_checker->getActiveCollisionObjects())
    contact_checker->setCollisionObjectsTransform(link_name, link_transforms[link_name]);

  tesseract_collision::ContactResultMap contact_map;
  contact_checker->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  return contact_map;
}

std::vector<ompl::base::ScopedState<>> createOMPLStates(const tesseract_kinematics::IKSolutions& joint_states,
                                                        ompl::base::SpaceInformationPtr si)
{
  std::vector<ompl::base::ScopedState<>> states;
  states.reserve(joint_states.size());

  for (const auto& js : joint_states)
  {
    ompl::base::ScopedState<> state(si->getStateSpace());
    for (unsigned i = 0; i < js.size(); ++i)
      state[i] = js[static_cast<Eigen::Index>(i)];

    states.push_back(state);
  }

  return states;
}

tesseract_kinematics::IKSolutions getValidIKSolutions(const Eigen::Isometry3d& cartesian_waypoint,
                                                      const ManipulatorInfo& mi,
                                                      tesseract_environment::Environment::ConstPtr env)
{
  if (mi.manipulator.empty())
    throw std::runtime_error("OMPL: manipulator is empty!");

  if (mi.tcp_frame.empty())
    throw std::runtime_error("OMPL: tcp_frame is empty!");

  if (mi.working_frame.empty())
    throw std::runtime_error("OMPL: working_frame is empty!");

  // Get the kinematics group for solving IK
  tesseract_kinematics::KinematicGroup::ConstPtr manip =
      env->getKinematicGroup(mi.manipulator, mi.manipulator_ik_solver);

  /** Solve IK
   *  @todo Need to add Descartes pose sample to ompl profile
   */
  Eigen::Isometry3d tcp_offset = env->findTCPOffset(mi);
  Eigen::Isometry3d tcp_frame_cwp = cartesian_waypoint * tcp_offset.inverse();
  tesseract_kinematics::KinGroupIKInput ik_input(tcp_frame_cwp, mi.working_frame, mi.tcp_frame);
  const tesseract_kinematics::IKSolutions joint_solutions =
      manip->calcInvKin({ ik_input }, Eigen::VectorXd::Zero(manip->numJoints()));

  // Check collision
  std::vector<tesseract_collision::ContactResultMap> contact_map_vec(static_cast<std::size_t>(joint_solutions.size()));
  tesseract_common::KinematicLimits limits = manip->getLimits();
  tesseract_kinematics::IKSolutions valid_solutions;
  valid_solutions.reserve(joint_solutions.size());
  for (std::size_t i = 0; i < joint_solutions.size(); ++i)
  {
    Eigen::VectorXd solution = joint_solutions[i];

    // Check limits
    if (tesseract_common::satisfiesPositionLimits(solution, limits.joint_limits))
    {
      tesseract_common::enforcePositionLimits(solution, limits.joint_limits);
    }
    else
    {
      CONSOLE_BRIDGE_logDebug("In OMPLDefaultPlanProfile: Goal state has invalid bounds");
      continue;
    }

    // Get discrete contact manager for testing provided start and end position
    // This is required because collision checking happens in motion validators now
    // instead of the isValid function to avoid unnecessary collision checks.
    contact_map_vec[i] = checkCollision(solution, manip, env->getDiscreteContactManager());
    if (contact_map_vec[i].empty())
    {
      valid_solutions.push_back(solution);

      auto redundant_solutions = tesseract_kinematics::getRedundantSolutions<double>(
          solution, limits.joint_limits, manip->getRedundancyCapableJointIndices());

      valid_solutions.insert(valid_solutions.end(), redundant_solutions.begin(), redundant_solutions.end());
    }
  }

  if (valid_solutions.empty())
  {
    for (std::size_t i = 0; i < contact_map_vec.size(); i++)
      for (const auto& contact_vec : contact_map_vec[i])
        for (const auto& contact : contact_vec.second)
          CONSOLE_BRIDGE_logError(("Solution: " + std::to_string(i) + "  Links: " + contact.link_names[0] + ", " +
                                   contact.link_names[1] + "  Distance: " + std::to_string(contact.distance))
                                      .c_str());
    throw std::runtime_error("In OMPLDefaultPlanProfile: All goal states are either in collision or outside limits");
  }

  return valid_solutions;
}

ompl::base::ScopedState<> createState(Eigen::VectorXd joint_waypoint,
                                      tesseract_environment::Environment::ConstPtr env,
                                      tesseract_kinematics::JointGroup::ConstPtr manip,
                                      ompl::base::StateSpacePtr ss)
{
  const auto dof = manip->numJoints();
  tesseract_common::KinematicLimits limits = manip->getLimits();

  if (tesseract_common::satisfiesPositionLimits(joint_waypoint, limits.joint_limits))
    tesseract_common::enforcePositionLimits(joint_waypoint, limits.joint_limits);
  else
    throw std::runtime_error("Start state is outside limits");

  // Get discrete contact manager for testing provided start and end position
  // This is required because collision checking happens in motion validators now
  // instead of the isValid function to avoid unnecessary collision checks.
  tesseract_collision::ContactResultMap contact_map =
      checkCollision(joint_waypoint, manip, env->getDiscreteContactManager());
  if (!contact_map.empty())
  {
    std::stringstream ss;
    ss << "Start state is in collision";
    for (const auto& contact_vec : contact_map)
      for (const auto& contact : contact_vec.second)
        ss << "\n\tLinks: " << contact.link_names[0] << ", " << contact.link_names[1]
           << " Distance: " << contact.distance;
    throw std::runtime_error(ss.str());
  }

  ompl::base::ScopedState<> start_state(ss);
  for (unsigned i = 0; i < dof; ++i)
    start_state[i] = joint_waypoint[i];

  return start_state;
}

OMPLCompositeProfileData OMPLCompositeProfileRVSS::create(const CompositeInstruction& instruction,
                                                          tesseract_environment::Environment::ConstPtr env) const
{
  if (instruction.size() > 1)
    throw std::runtime_error("The composite instruction contains more than one instruction. The composite instruction "
                             "can "
                             "only contain one child and it must be a PlanInstruction type");

  const PlanInstruction& goal_instruction = instruction.at(0).as<PlanInstruction>();
  if (goal_instruction.getPlanType() != PlanInstructionType::FREESPACE)
    throw std::runtime_error("Move types other than freespace are not currently supported");

  // Get the joint group information from the composite instruction and environment
  const tesseract_kinematics::JointGroup::ConstPtr joint_group =
      env->getJointGroup(instruction.getManipulatorInfo().manipulator);
  const std::vector<std::string> joint_names = joint_group->getJointNames();
  const Eigen::MatrixX2d limits = joint_group->getLimits().joint_limits;
  const Eigen::Index dof = joint_group->numJoints();

  // Set up the extractor
  OMPLStateExtractor extractor = [dof](const ompl::base::State* state) -> Eigen::Map<Eigen::VectorXd> {
    return tesseract_planning::RealVectorStateSpaceExtractor(state, static_cast<unsigned>(dof));
  };

  // Construct the real vector state space
  auto rss = std::make_shared<ompl::base::RealVectorStateSpace>();
  for (unsigned i = 0; i < dof; ++i)
  {
    rss->addDimension(joint_names[i], limits(i, 0), limits(i, 1));
  }

  // Create the state sampler
  if (state_sampler_allocator)
  {
    rss->setStateSamplerAllocator(state_sampler_allocator);
  }
  else
  {
    Eigen::VectorXd weights = Eigen::VectorXd::Ones(dof);
    rss->setStateSamplerAllocator(
        [weights, limits, this](const ompl::base::StateSpace* state_space) -> ompl::base::StateSamplerPtr {
          return std::make_shared<WeightedRealVectorStateSampler>(state_space, weights, limits, this->rng_seed);
        });
  }

  // Setup Longest Valid Segment
  processLongestValidSegment(rss, collision_check_config);

  // Create the SimpleSetup from the state space
  auto simple_setup = std::make_shared<ompl::geometric::SimpleSetup>(rss);

  // Setup state checking functionality
  ompl::base::StateValidityCheckerPtr custom_validity_checker{ nullptr };
  {
    auto csvc = std::make_shared<CompoundStateValidator>();
    if (state_validator_allocator != nullptr)
    {
      custom_validity_checker = state_validator_allocator(simple_setup->getSpaceInformation());
      csvc->addStateValidator(custom_validity_checker);
    }
    else
    {
      switch (collision_check_config.type)
      {
        case tesseract_collision::CollisionEvaluatorType::DISCRETE:
        case tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE:
        {
          auto svc = std::make_shared<StateCollisionValidator>(
              simple_setup->getSpaceInformation(), *env, joint_group, collision_check_config, extractor);
          csvc->addStateValidator(svc);
          break;
        }
        default:
          CONSOLE_BRIDGE_logDebug("DOING NOTHING HERE");
      }
      simple_setup->setStateValidityChecker(csvc);
    }
  }

  // Setup motion validation (i.e. collision checking)
  if (motion_validator_allocator != nullptr)
  {
    simple_setup->getSpaceInformation()->setMotionValidator(
        motion_validator_allocator(simple_setup->getSpaceInformation()));
  }
  else
  {
    ompl::base::MotionValidatorPtr mv{ nullptr };

    switch (collision_check_config.type)
    {
      case tesseract_collision::CollisionEvaluatorType::DISCRETE:
      case tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE:
      {
        // Collision checking is preformed using the state validator which this calls.
        mv = std::make_shared<DiscreteMotionValidator>(simple_setup->getSpaceInformation());
        break;
      }
      case tesseract_collision::CollisionEvaluatorType::CONTINUOUS:
      case tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS:
      {
        mv = std::make_shared<ContinuousMotionValidator>(simple_setup->getSpaceInformation(),
                                                         custom_validity_checker,
                                                         *env,
                                                         joint_group,
                                                         collision_check_config,
                                                         extractor);
        break;
      }
      default:
        CONSOLE_BRIDGE_logDebug("Nothing doing");
    }

    simple_setup->getSpaceInformation()->setMotionValidator(mv);
  }

  // make sure the planners run until the time limit, and get the best possible solution
  if (optimization_objective_allocator)
  {
    simple_setup->getProblemDefinition()->setOptimizationObjective(
        optimization_objective_allocator(simple_setup->getSpaceInformation()));
  }

  // Add the start waypoint
  {
    // Get the start waypoint
    Waypoint start_waypoint{ NullWaypoint() };
    tesseract_common::ManipulatorInfo start_mi = instruction.getManipulatorInfo();
    try
    {
      const auto& pi = instruction.getStartInstruction().as<PlanInstruction>();
      start_waypoint = pi.getWaypoint();
      start_mi = start_mi.getCombined(pi.getManipulatorInfo());
    }
    catch (const std::bad_cast& ex)
    {
      Eigen::VectorXd current_jv = env->getCurrentJointValues();
      start_waypoint = StateWaypoint(joint_names, current_jv);
    }

    // Add the start state(s)
    if (isCartesianWaypoint(start_waypoint))
    {
      const CartesianWaypoint& cw = start_waypoint.as<CartesianWaypoint>();
      tesseract_kinematics::IKSolutions sols = getValidIKSolutions(cw, start_mi, env);
      auto states = createOMPLStates(sols, simple_setup->getSpaceInformation());

      // Add the states to the SimpleSetup
      std::for_each(states.begin(), states.end(), [&simple_setup](const ompl::base::ScopedState<>& state) {
        simple_setup->addStartState(state);
      });
    }
    else if (isJointWaypoint(start_waypoint))
    {
      const JointWaypoint& jw = start_waypoint.as<JointWaypoint>();
      simple_setup->addStartState(createState(jw, env, joint_group, simple_setup->getStateSpace()));
    }
    else
    {
      throw std::runtime_error("Unsupported start waypoint type");
    }
  }

  // Add the goal waypoint
  {
    tesseract_common::ManipulatorInfo goal_mi =
        goal_instruction.getManipulatorInfo().getCombined(instruction.getManipulatorInfo());

    // Add the goal state
    if (isCartesianWaypoint(goal_instruction.getWaypoint()))
    {
      const CartesianWaypoint& cw = goal_instruction.getWaypoint().as<CartesianWaypoint>();
      tesseract_kinematics::IKSolutions sols = getValidIKSolutions(cw, goal_mi, env);
      auto states = createOMPLStates(sols, simple_setup->getSpaceInformation());

      auto goal_states = std::make_shared<ompl::base::GoalStates>(simple_setup->getSpaceInformation());
      std::for_each(states.begin(), states.end(), [&goal_states](const ompl::base::ScopedState<>& state) {
        goal_states->addState(state);
      });

      simple_setup->setGoal(goal_states);
    }
    else if (isJointWaypoint(goal_instruction.getWaypoint()))
    {
      const JointWaypoint& jw = goal_instruction.getWaypoint().as<JointWaypoint>();
      simple_setup->setGoalState(createState(jw, env, joint_group, simple_setup->getStateSpace()));
    }
    else
    {
      throw std::runtime_error("Unsupported goal waypoint type");
    }
  }

  // Return the composite data
  return std::make_tuple(simple_setup, extractor);
}

}  // namespace tesseract_planning
