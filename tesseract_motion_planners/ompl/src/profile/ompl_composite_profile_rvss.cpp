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
std::any OMPLCompositeProfileRVSS::create(const CompositeInstruction& instruction,
                                          const tesseract_environment::Environment& env) const
{
  for (const Instruction& inst : instruction)
  {
    const auto& goal_instruction = inst.as<PlanInstruction>();
    if (goal_instruction.getPlanType() != PlanInstructionType::FREESPACE)
      throw std::runtime_error("Only freespace plan instruction types are currently supported");
  }

  // Get the joint group information from the composite instruction and environment
  const tesseract_kinematics::JointGroup::ConstPtr joint_group =
      env.getJointGroup(instruction.getManipulatorInfo().manipulator);
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
              simple_setup->getSpaceInformation(), env, joint_group, collision_check_config, extractor);
          csvc->addStateValidator(svc);
          break;
        }
        default:
          CONSOLE_BRIDGE_logDebug("No collision state validator added");
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
                                                         env,
                                                         joint_group,
                                                         collision_check_config,
                                                         extractor);
        break;
      }
      default:
        CONSOLE_BRIDGE_logDebug("No collision validator added");
    }

    simple_setup->getSpaceInformation()->setMotionValidator(mv);
  }

  // make sure the planners run until the time limit, and get the best possible solution
  if (optimization_objective_allocator)
  {
    simple_setup->getProblemDefinition()->setOptimizationObjective(
        optimization_objective_allocator(simple_setup->getSpaceInformation()));
  }

  // Return the composite data
  return std::make_tuple(simple_setup, extractor);
}

}  // namespace tesseract_planning
