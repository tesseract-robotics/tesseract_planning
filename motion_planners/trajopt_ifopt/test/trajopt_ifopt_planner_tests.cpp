#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/common/resource_locator.h>

#include <tesseract/kinematics/joint_group.h>

#include <tesseract/environment/environment.h>

#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/joint_waypoint.h>
#include <tesseract/command_language/state_waypoint.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/utils.h>

#include <tesseract/motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#ifdef TRAJOPT_SQP_HAS_PIQP
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_piqp_solver_profile.h>
#endif
#include <tesseract/motion_planners/simple/interpolation.h>
#include <tesseract/motion_planners/types.h>

using namespace tesseract::environment;
using namespace tesseract::motion_planners;
using namespace tesseract::command_language;

static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";

class TesseractPlanningTrajOptIfoptUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  tesseract::common::ManipulatorInfo manip;
  Eigen::VectorXd q0{ Eigen::VectorXd::Zero(7) };
  Eigen::VectorXd q1{ Eigen::VectorXd::Zero(7) };

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();
    std::filesystem::path urdf_path(
        locator->locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath());
    std::filesystem::path srdf_path(
        locator->locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath());
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;
    manip.tcp_frame = "tool0";
    manip.working_frame = "base_link";
    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "KDLInvKinChainLMA";
    q0 << 0, 0, 0, -1.57, 0, 0, 0;
    q1 << 0, 0, 0, 1.57, 0, 0, 0;
  }

  const std::vector<tesseract::common::JointId>& jointIds() const
  {
    return env_->getJointGroup(manip.manipulator)->getJointIds();
  }

  /** @brief Interpolate a start-to-goal program into the seed the planner expects */
  CompositeInstruction interpolate(const CompositeInstruction& program) const
  {
    return generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 10);
  }

  /** @brief Solve an interpolated program with the given solver profile */
  PlannerResponse solve(const CompositeInstruction& program,
                        const TrajOptIfoptSolverProfile::ConstPtr& solver_profile,
                        const tesseract::common::ProfileDictionary::Ptr& profiles) const
  {
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", solver_profile);

    PlannerRequest request;
    request.instructions = program;
    request.env = env_;
    request.profiles = profiles;

    TrajOptIfoptMotionPlanner planner(TRAJOPT_IFOPT_DEFAULT_NAMESPACE);
    return planner.solve(request);
  }
};

namespace
{
Eigen::VectorXd resultPosition(const PlannerResponse& response, std::size_t index)
{
  auto move_instructions = response.results.flatten(&moveFilter);
  return getJointPosition(move_instructions.at(index).get().as<MoveInstructionPoly>().getWaypoint());
}

std::vector<TrajOptIfoptSolverProfile::ConstPtr> solverProfiles()
{
  std::vector<TrajOptIfoptSolverProfile::ConstPtr> profiles;
  profiles.push_back(std::make_shared<TrajOptIfoptOSQPSolverProfile>());
#ifdef TRAJOPT_SQP_HAS_PIQP
  profiles.push_back(std::make_shared<TrajOptIfoptPIQPSolverProfile>());
#endif
  return profiles;
}
}  // namespace

// Fixed StateWaypoints start and mid with the joint constraint and cost disabled: nothing but the variable bounds
// holds them. The mid state is off the start-goal line, so smoothing pulls it.
TEST_F(TesseractPlanningTrajOptIfoptUnit, FixedStateWaypointsHeldWithoutJointConstraint)  // NOLINT
{
  Eigen::VectorXd qm = Eigen::VectorXd::Zero(7);
  qm(0) = 1.0;

  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.push_back(MoveInstruction(StateWaypoint(jointIds(), q0), MoveInstructionType::FREESPACE, "TEST_PROFILE"));
  program.push_back(MoveInstruction(StateWaypoint(jointIds(), qm), MoveInstructionType::FREESPACE, "TEST_PROFILE"));
  program.push_back(MoveInstruction(JointWaypoint(jointIds(), q1), MoveInstructionType::FREESPACE, "TEST_PROFILE"));

  auto move_profile = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
  move_profile->joint_constraint_config.enabled = false;
  move_profile->joint_cost_config.enabled = false;
  auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();
  profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", move_profile);
  profiles->addProfile(
      TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", std::make_shared<TrajOptIfoptDefaultCompositeProfile>());

  // The mid instruction is the one state waypoint after the start in the interpolated program
  const CompositeInstruction interpolated = interpolate(program);
  std::size_t mid_index = 0;
  {
    auto move_instructions = interpolated.flatten(&moveFilter);
    for (std::size_t i = 1; i < move_instructions.size(); ++i)
    {
      if (move_instructions[i].get().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint())
        mid_index = i;
    }
  }
  ASSERT_GT(mid_index, 0U);

  for (const auto& solver : solverProfiles())
  {
    PlannerResponse response = solve(interpolated, solver, profiles);
    EXPECT_TRUE(response.successful);
    EXPECT_TRUE((resultPosition(response, 0).array() == q0.array()).all());
    EXPECT_TRUE((resultPosition(response, mid_index).array() == qm.array()).all());
  }
}

// Constrained JointWaypoint start and goal with all profiles default.
TEST_F(TesseractPlanningTrajOptIfoptUnit, DefaultJointWaypointsHeld)  // NOLINT
{
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.push_back(MoveInstruction(JointWaypoint(jointIds(), q0), MoveInstructionType::FREESPACE, "TEST_PROFILE"));
  program.push_back(MoveInstruction(JointWaypoint(jointIds(), q1), MoveInstructionType::FREESPACE, "TEST_PROFILE"));

  const CompositeInstruction interpolated = interpolate(program);
  auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();
  for (const auto& solver : solverProfiles())
  {
    PlannerResponse response = solve(interpolated, solver, profiles);
    EXPECT_TRUE(response.successful);
    const std::size_t last = response.results.flatten(&moveFilter).size() - 1;
    EXPECT_TRUE((resultPosition(response, 0).array() == q0.array()).all());
    EXPECT_TRUE((resultPosition(response, last).array() == q1.array()).all());
  }
}

// A fixed start outside the joint limits that reaches the planner is clamped by its variable bounds. The joint
// constraint targets the requested position, so the solve fails instead of returning a plan from a shifted start.
TEST_F(TesseractPlanningTrajOptIfoptUnit, FixedStartOutsideJointLimitsStillFails)  // NOLINT
{
  const double upper = env_->getJointGroup(manip.manipulator)->getLimits().joint_limits(0, 1);
  Eigen::VectorXd q_out = q0;
  q_out(0) = upper + 0.05;

  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.push_back(MoveInstruction(JointWaypoint(jointIds(), q0), MoveInstructionType::FREESPACE, "TEST_PROFILE"));
  program.push_back(MoveInstruction(JointWaypoint(jointIds(), q1), MoveInstructionType::FREESPACE, "TEST_PROFILE"));

  // Interpolation clamps to the joint limits, so move the start outside them afterwards
  CompositeInstruction interpolated = interpolate(program);
  auto move_instructions = interpolated.flatten(&moveFilter);
  ASSERT_TRUE(setJointPosition(move_instructions.front().get().as<MoveInstructionPoly>().getWaypoint(), q_out));

  auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();
  for (const auto& solver : solverProfiles())
  {
    PlannerResponse response = solve(interpolated, solver, profiles);
    EXPECT_FALSE(response.successful);
  }
}
