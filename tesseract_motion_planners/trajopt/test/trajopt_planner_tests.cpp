/**
  These tests test the TrajOptArrayPlanner and the TrajOptFreespacePlanner. They primarily check that the correct types
  of costs and constraints are added when the flags like smooth_velocity are specified. However they are not foolproof.
  They only check that at least one term of the correct type is in the cost or constraint vector. If there should be
  more than one, then it might not be caught. This could be improved in the future, but it is better than nothing.

  Additional features that could be tested in the future
  * Configuration costs added correctly
  * Intermediate waypoints added correctly to freespace
  * coeffs set correctly
  * init info is set correctly
  * Seed trajectory is set correctly
  * callbacks are added correctly
  * Number of steps are obeyed for freespace
  * continuous collision checking flag set correctly

  Last updated: July 15, 2019
  Matthew Powelson
*/

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>

// These contain the definitions of the cost types
#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt/collision_terms.hpp>
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/serialize.h>
#include <tesseract_motion_planners/trajopt/deserialize.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

const int NUM_STEPS = 7;

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_planning;

static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";

class TesseractPlanningTrajoptUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  tesseract_common::ManipulatorInfo manip;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract_common::TesseractSupportResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;
    manip.tcp_frame = "tool0";
    manip.working_frame = "base_link";
    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "KDLInvKinChainLMA";
  }
};

namespace tesseract_tests
{
/** @brief Checks if the vector of base class types contains at least one instance of the Derived type */
template <class Base, class Derived>
bool vectorContainsType(std::vector<Base> vector)
{
  bool contains_type = false;
  for (const Base& unit : vector)
  {
    auto* test = dynamic_cast<Derived*>(unit.get());
    contains_type |= (test != nullptr);
  }
  return contains_type;
}

/** @brief Checks if the object passed in is of the derived type */
template <class Base, class Derived>
bool objectIsType(Base unit)
{
  auto* test = dynamic_cast<Derived*>(unit.get());
  bool contains_type = (test != nullptr);

  return contains_type;
}

}  // namespace tesseract_tests

// This test checks that the boolean flags are adding the correct costs for smoothing and collision
TEST_F(TesseractPlanningTrajoptUnit, TrajoptPlannerBooleanFlagsJointJoint)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip.manipulator);
  std::vector<std::string> joint_names = joint_group->getJointNames();
  auto cur_state = env_->getState();

  // Specify a JointWaypoint as the start
  JointWaypointPoly wp1{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(7)) };
  wp1.getPosition() << 0, 0, 0, -1.57, 0, 0, 0;

  // Specify a Joint Waypoint as the finish
  JointWaypointPoly wp2{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(7)) };
  wp2.getPosition() << 0, 0, 0, 1.57, 0, 0, 0;

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program =
      generateInterpolatedProgram(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.env_state = cur_state;
  request.profiles = profiles;

  // Loop over all combinations of these 4. 0001, 0010, 0011, ... , 1111
  for (uint8_t byte = 0; byte < 16; byte++)
  {
    auto t1 = static_cast<bool>(byte & 0x1);
    auto t2 = static_cast<bool>(byte & 0x2);
    auto t3 = static_cast<bool>(byte & 0x4);
    auto t4 = static_cast<bool>(byte & 0x8);

    composite_profile->smooth_velocities = t1;
    composite_profile->smooth_accelerations = t2;
    composite_profile->smooth_jerks = t3;
    composite_profile->collision_constraint_config.enabled = t4;
    composite_profile->collision_cost_config.enabled = t4;

    std::shared_ptr<trajopt::ProblemConstructionInfo> pci = test_planner.createProblem(request);

    trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointVelEqCost>(problem->getCosts())), t1);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointAccEqCost>(problem->getCosts())), t2);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointJerkEqCost>(problem->getCosts())), t3);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::CollisionCost>(problem->getCosts())), t4);
  }
}

// This test tests freespace motion b/n 2 joint waypoints
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespaceJointJoint)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip.manipulator);
  std::vector<std::string> joint_names = joint_group->getJointNames();
  auto cur_state = env_->getState();

  // Specify a JointWaypoint as the start
  JointWaypointPoly wp1{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(7)) };
  wp1.getPosition() << 0, 0, 0, -1.57, 0, 0, 0;

  // Specify a Joint Waypoint as the finish
  JointWaypointPoly wp2{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(7)) };
  wp2.getPosition() << 0, 0, 0, 1.57, 0, 0, 0;

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program =
      generateInterpolatedProgram(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.env_state = cur_state;
  request.profiles = profiles;

  {
    std::shared_ptr<trajopt::ProblemConstructionInfo> pci = test_planner.createProblem(request);

    trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        problem->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        problem->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(problem->getCosts())));
    EXPECT_FALSE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(problem->getCosts())));
  }
  {
    plan_profile->term_type = trajopt::TermType::TT_COST;

    std::shared_ptr<trajopt::ProblemConstructionInfo> pci = test_planner.createProblem(request);

    trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        problem->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        problem->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(problem->getCosts())));
    EXPECT_FALSE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(problem->getCosts())));
  }
}

// This test tests freespace motion b/n 1 joint waypoint and 1 cartesian waypoint
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespaceJointCart)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip.manipulator);
  std::vector<std::string> joint_names = joint_group->getJointNames();
  auto cur_state = env_->getState();

  // Specify a JointWaypoint as the start
  JointWaypointPoly wp1{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(7)) };
  wp1.getPosition() << 0, 0, 0, -1.57, 0, 0, 0;

  // Specify a CartesianWaypoint as the finish
  CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(-.20, .4, 0.2) *
                                               Eigen::Quaterniond(0, 0, 1.0, 0)) };

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program =
      generateInterpolatedProgram(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.env_state = cur_state;
  request.profiles = profiles;

  {
    std::shared_ptr<trajopt::ProblemConstructionInfo> pci = test_planner.createProblem(request);

    trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        problem->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        problem->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(problem->getCosts())));
    EXPECT_FALSE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(problem->getCosts())));
  }

  {
    plan_profile->term_type = trajopt::TermType::TT_COST;
    std::shared_ptr<trajopt::ProblemConstructionInfo> pci = test_planner.createProblem(request);

    trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        problem->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        problem->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(problem->getCosts())));
    EXPECT_TRUE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(problem->getCosts())));
  }
}

// This test tests freespace motion b/n 1 cartesian waypoint and 1 joint waypoint
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespaceCartJoint)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto joint_group = env_->getJointGroup(manip.manipulator);
  std::vector<std::string> joint_names = joint_group->getJointNames();
  auto cur_state = env_->getState();

  // Specify a JointWaypoint as the start
  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(-.20, .4, 0.2) *
                                               Eigen::Quaterniond(0, 0, 1.0, 0)) };

  // Specify a Joint Waypoint as the finish
  JointWaypointPoly wp2{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(7)) };
  wp2.getPosition() << 0, 0, 0, -1.57, 0, 0, 0;

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE");
  start_instruction.getManipulatorInfo().working_frame = "base_link";

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program =
      generateInterpolatedProgram(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.env_state = cur_state;
  request.profiles = profiles;

  {
    std::shared_ptr<trajopt::ProblemConstructionInfo> pci = test_planner.createProblem(request);

    trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        problem->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        problem->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(problem->getCosts())));
    EXPECT_FALSE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(problem->getCosts())));
  }

  {
    plan_profile->term_type = trajopt::TermType::TT_COST;
    std::shared_ptr<trajopt::ProblemConstructionInfo> pci = test_planner.createProblem(request);

    trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        problem->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        problem->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(problem->getCosts())));
    EXPECT_TRUE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(problem->getCosts())));
  }
}

// This test tests freespace motion b/n 2 cartesian waypoints
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespaceCartCart)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto joint_group = env_->getJointGroup(manip.manipulator);
  auto cur_state = env_->getState();

  // Specify a CartesianWaypoint as the start
  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(-.20, .4, 0.2) *
                                               Eigen::Quaterniond(0, 0, 1.0, 0)) };

  // Specify a CartesianWaypoint as the finish
  CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(-.20, .4, 0.2) *
                                               Eigen::Quaterniond(0, 0, 1.0, 0)) };

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE");
  start_instruction.getManipulatorInfo().working_frame = "base_link";

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE");
  plan_f1.getManipulatorInfo().working_frame = "base_link";

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program =
      generateInterpolatedProgram(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.env_state = cur_state;
  request.profiles = profiles;

  {
    std::shared_ptr<trajopt::ProblemConstructionInfo> pci = test_planner.createProblem(request);

    trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        problem->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        problem->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(problem->getCosts())));
    EXPECT_FALSE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(problem->getCosts())));
  }
  {
    plan_profile->term_type = trajopt::TermType::TT_COST;
    std::shared_ptr<trajopt::ProblemConstructionInfo> pci = test_planner.createProblem(request);

    trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        problem->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        problem->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(problem->getCosts())));
    EXPECT_TRUE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(problem->getCosts())));
  }
}

// This test checks that the boolean flags are adding the correct costs for smoothing, collision, and cartesian cnts
// are / added correctly
TEST_F(TesseractPlanningTrajoptUnit, TrajoptPlannerBooleanFlagsCartCart)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto joint_group = env_->getJointGroup(manip.manipulator);
  auto cur_state = env_->getState();

  // Specify a JointWaypoint as the start
  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(-.20, .4, 0.8) *
                                               Eigen::Quaterniond(0, 0, 1.0, 0)) };

  // Specify a Joint Waypoint as the finish
  CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(.20, .4, 0.8) *
                                               Eigen::Quaterniond(0, 0, 1.0, 0)) };

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE");
  start_instruction.getManipulatorInfo().working_frame = "base_link";

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE");
  plan_f1.getManipulatorInfo().working_frame = "base_link";

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program =
      generateInterpolatedProgram(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.env_state = cur_state;
  request.profiles = profiles;

  std::shared_ptr<trajopt::ProblemConstructionInfo> pci;
  trajopt::TrajOptProb::Ptr problem;

  // Loop over all combinations of these 4. 0001, 0010, 0011, ... , 1111
  for (uint8_t byte = 0; byte < 16; byte++)
  {
    auto t1 = static_cast<bool>(byte & 0x1);
    auto t2 = static_cast<bool>(byte & 0x2);
    auto t3 = static_cast<bool>(byte & 0x4);
    auto t4 = static_cast<bool>(byte & 0x8);

    composite_profile->smooth_velocities = t1;
    composite_profile->smooth_accelerations = t2;
    composite_profile->smooth_jerks = t3;
    composite_profile->collision_constraint_config.enabled = t4;
    composite_profile->collision_cost_config.enabled = t4;

    pci = test_planner.createProblem(request);

    problem = trajopt::ConstructProblem(*pci);

    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointVelEqCost>(problem->getCosts())), t1);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointAccEqCost>(problem->getCosts())), t2);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointJerkEqCost>(problem->getCosts())), t3);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::CollisionCost>(problem->getCosts())), t4);
  }
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
      problem->getConstraints())));
  EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
      problem->getConstraints())));
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(problem->getCosts())));
  EXPECT_FALSE(
      (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(problem->getCosts())));
}

// This test checks that the terms are being added correctly for joint cnts
TEST_F(TesseractPlanningTrajoptUnit, TrajoptArrayJointConstraint)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto joint_group = env_->getJointGroup(manip.manipulator);
  std::vector<std::string> joint_names = joint_group->getJointNames();
  auto cur_state = env_->getState();

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);

  // These specify the series of points to be optimized
  for (int ind = 0; ind < NUM_STEPS; ind++)
  {
    // Specify a Joint Waypoint as the finish
    JointWaypointPoly wp{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(7)) };
    wp.getPosition() << 0, 0, 0, -1.57 + ind * 0.1, 0, 0, 0;
    MoveInstruction plan_f(wp, MoveInstructionType::FREESPACE, "TEST_PROFILE");
    program.appendMoveInstruction(plan_f);
  }

  // Create a seed
  CompositeInstruction interpolated_program =
      generateInterpolatedProgram(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.env_state = cur_state;
  request.profiles = profiles;

  std::shared_ptr<trajopt::ProblemConstructionInfo> pci = test_planner.createProblem(request);

  trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

  EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
      problem->getConstraints())));
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
      problem->getConstraints())));
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(problem->getCosts())));
  EXPECT_FALSE(
      (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(problem->getCosts())));
}

// This test checks that the terms are being added correctly for joint costs
TEST_F(TesseractPlanningTrajoptUnit, TrajoptArrayJointCost)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto joint_group = env_->getJointGroup(manip.manipulator);
  const std::vector<std::string>& joint_names = joint_group->getJointNames();
  auto cur_state = env_->getState();

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);

  // These specify the series of points to be optimized
  for (int ind = 0; ind < NUM_STEPS; ind++)
  {
    // Specify a Joint Waypoint as the finish
    JointWaypointPoly wp{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(7)) };
    wp.getPosition() << 0, 0, 0, -1.57 + ind * 0.1, 0, 0, 0;
    MoveInstruction plan_f(wp, MoveInstructionType::FREESPACE, "TEST_PROFILE");
    program.appendMoveInstruction(plan_f);
  }

  // Create a seed
  CompositeInstruction interpolated_program =
      generateInterpolatedProgram(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  plan_profile->term_type = trajopt::TermType::TT_COST;  // Everything associated with profile is now added as a cost

  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.env_state = cur_state;
  request.profiles = profiles;

  std::shared_ptr<trajopt::ProblemConstructionInfo> pci = test_planner.createProblem(request);

  trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
      problem->getConstraints())));
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
      problem->getConstraints())));
  EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(problem->getCosts())));
  EXPECT_FALSE(
      (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(problem->getCosts())));
}

TEST(TesseractPlanningTrajoptSerializeUnit, SerializeTrajoptDefaultCompositeToXml)  // NOLINT
{
  // Write program to file
  TrajOptDefaultCompositeProfile comp_profile;
  CollisionCostConfig collision_cost_config;
  comp_profile.collision_cost_config = collision_cost_config;

  CollisionConstraintConfig collision_constraint_config;
  comp_profile.collision_constraint_config = collision_constraint_config;

  comp_profile.velocity_coeff = Eigen::VectorXd::Ones(6) * 10;
  comp_profile.acceleration_coeff = Eigen::VectorXd::Ones(6) * 10;
  comp_profile.jerk_coeff = Eigen::VectorXd::Ones(6) * 10;

  comp_profile.smooth_velocities = false;

  EXPECT_TRUE(toXMLFile(comp_profile, tesseract_common::getTempPath() + "trajopt_default_composite_example_input.xml"));

  // Import file
  TrajOptDefaultCompositeProfile imported_comp_profile =
      trajOptCompositeFromXMLFile(tesseract_common::getTempPath() + "trajopt_default_composite_example_input.xml");

  // Re-write file and compare changed from default term
  EXPECT_TRUE(toXMLFile(imported_comp_profile,
                        tesseract_common::getTempPath() + "trajopt_default_composite_example_input2.xml"));
  EXPECT_TRUE(comp_profile.smooth_velocities == imported_comp_profile.smooth_velocities);
}

TEST(TesseractPlanningTrajoptSerializeUnit, SerializeTrajoptDefaultPlanToXml)  // NOLINT
{
  // Write program to file
  TrajOptDefaultPlanProfile plan_profile;
  plan_profile.cartesian_coeff = Eigen::VectorXd::Ones(6) * 10;
  plan_profile.joint_coeff = Eigen::VectorXd::Ones(6) * 9;
  plan_profile.term_type = trajopt::TermType::TT_COST;

  EXPECT_TRUE(toXMLFile(plan_profile, tesseract_common::getTempPath() + "trajopt_default_plan_example_input.xml"));

  // Import file
  TrajOptDefaultPlanProfile imported_plan_profile = trajOptPlanFromXMLFile(tesseract_common::getTempPath() + "trajopt_"
                                                                                                             "default_"
                                                                                                             "plan_"
                                                                                                             "example_"
                                                                                                             "input."
                                                                                                             "xml");

  // Re-write file and compare changed from default term
  EXPECT_TRUE(
      toXMLFile(imported_plan_profile, tesseract_common::getTempPath() + "trajopt_default_plan_example_input2.xml"));
  EXPECT_TRUE(plan_profile.term_type == imported_plan_profile.term_type);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
