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
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/cereal_serialization.h>
#include <tesseract_common/unit_test_utils.h>

#include <tesseract_kinematics/core/joint_group.h>

#include <tesseract_scene_graph/scene_state.h>

#include <tesseract_environment/environment.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/cereal_serializatiion.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_common/resource_locator.h>

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
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();
    std::filesystem::path urdf_path(
        locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath());
    std::filesystem::path srdf_path(
        locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath());
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

  // Specify a JointWaypoint as the start
  JointWaypoint wp1{ joint_names, Eigen::VectorXd::Zero(7) };
  wp1.getPosition() << 0, 0, 0, -1.57, 0, 0, 0;

  // Specify a Joint Waypoint as the finish
  JointWaypoint wp2{ joint_names, Eigen::VectorXd::Zero(7) };
  wp2.getPosition() << 0, 0, 0, 1.57, 0, 0, 0;

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.push_back(start_instruction);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  auto solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();

  // Serialization
  tesseract_common::testSerializationDerivedClass<tesseract_common::Profile, TrajOptDefaultMoveProfile>(move_profile,
                                                                                                        "trajopt_move_"
                                                                                                        "profile");
  tesseract_common::testSerializationDerivedClass<tesseract_common::Profile, TrajOptDefaultCompositeProfile>(
      composite_profile, "trajopt_composite_profile");
  tesseract_common::testSerializationDerivedClass<tesseract_common::Profile, TrajOptOSQPSolverProfile>(solver_profile,
                                                                                                       "trajopt_solver_"
                                                                                                       "profile");

  // Profile Dictionary
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", move_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", solver_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
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

  // Specify a JointWaypoint as the start
  JointWaypoint wp1{ joint_names, Eigen::VectorXd::Zero(7) };
  wp1.getPosition() << 0, 0, 0, -1.57, 0, 0, 0;

  // Specify a Joint Waypoint as the finish
  JointWaypoint wp2{ joint_names, Eigen::VectorXd::Zero(7) };
  wp2.getPosition() << 0, 0, 0, 1.57, 0, 0, 0;

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.push_back(start_instruction);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  auto solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", move_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", solver_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.profiles = profiles;

  {
    move_profile->cartesian_cost_config.enabled = false;
    move_profile->cartesian_constraint_config.enabled = true;
    move_profile->joint_cost_config.enabled = false;
    move_profile->joint_constraint_config.enabled = true;
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
    move_profile->cartesian_cost_config.enabled = true;
    move_profile->cartesian_constraint_config.enabled = false;
    move_profile->joint_cost_config.enabled = true;
    move_profile->joint_constraint_config.enabled = false;

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

  // Specify a JointWaypoint as the start
  JointWaypoint wp1{ joint_names, Eigen::VectorXd::Zero(7) };
  wp1.getPosition() << 0, 0, 0, -1.57, 0, 0, 0;

  // Specify a CartesianWaypoint as the finish
  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(-.20, .4, 0.2) *
                         Eigen::Quaterniond(0, 0, 1.0, 0) };

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.push_back(start_instruction);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  auto solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", move_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", solver_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.profiles = profiles;

  {
    move_profile->cartesian_cost_config.enabled = false;
    move_profile->cartesian_constraint_config.enabled = true;
    move_profile->joint_cost_config.enabled = false;
    move_profile->joint_constraint_config.enabled = true;
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
    move_profile->cartesian_cost_config.enabled = true;
    move_profile->cartesian_constraint_config.enabled = false;
    move_profile->joint_cost_config.enabled = true;
    move_profile->joint_constraint_config.enabled = false;
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
  auto joint_group = env_->getJointGroup(manip.manipulator);
  std::vector<std::string> joint_names = joint_group->getJointNames();

  // Specify a JointWaypoint as the start
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(-.20, .4, 0.2) *
                         Eigen::Quaterniond(0, 0, 1.0, 0) };

  // Specify a Joint Waypoint as the finish
  JointWaypoint wp2{ joint_names, Eigen::VectorXd::Zero(7) };
  wp2.getPosition() << 0, 0, 0, -1.57, 0, 0, 0;

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE");
  start_instruction.getManipulatorInfo().working_frame = "base_link";

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.push_back(start_instruction);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  auto solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", move_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", solver_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.profiles = profiles;

  {
    move_profile->cartesian_cost_config.enabled = false;
    move_profile->cartesian_constraint_config.enabled = true;
    move_profile->joint_cost_config.enabled = false;
    move_profile->joint_constraint_config.enabled = true;
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
    move_profile->cartesian_cost_config.enabled = true;
    move_profile->cartesian_constraint_config.enabled = false;
    move_profile->joint_cost_config.enabled = true;
    move_profile->joint_constraint_config.enabled = false;
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
  // Specify a CartesianWaypoint as the start
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(-.20, .4, 0.2) *
                         Eigen::Quaterniond(0, 0, 1.0, 0) };

  // Specify a CartesianWaypoint as the finish
  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(-.20, .4, 0.2) *
                         Eigen::Quaterniond(0, 0, 1.0, 0) };

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE");
  start_instruction.getManipulatorInfo().working_frame = "base_link";

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE");
  plan_f1.getManipulatorInfo().working_frame = "base_link";

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.push_back(start_instruction);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  auto solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", move_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", solver_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.profiles = profiles;

  {
    move_profile->cartesian_cost_config.enabled = false;
    move_profile->cartesian_constraint_config.enabled = true;
    move_profile->joint_cost_config.enabled = false;
    move_profile->joint_constraint_config.enabled = true;
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
    move_profile->cartesian_cost_config.enabled = true;
    move_profile->cartesian_constraint_config.enabled = false;
    move_profile->joint_cost_config.enabled = true;
    move_profile->joint_constraint_config.enabled = false;
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
  // Specify a JointWaypoint as the start
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(-.20, .4, 0.8) *
                         Eigen::Quaterniond(0, 0, 1.0, 0) };

  // Specify a Joint Waypoint as the finish
  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(.20, .4, 0.8) *
                         Eigen::Quaterniond(0, 0, 1.0, 0) };

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE");
  start_instruction.getManipulatorInfo().working_frame = "base_link";

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE");
  plan_f1.getManipulatorInfo().working_frame = "base_link";

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);
  program.push_back(start_instruction);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  auto solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", move_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", solver_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
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

    move_profile->cartesian_cost_config.enabled = false;
    move_profile->cartesian_constraint_config.enabled = true;
    move_profile->joint_cost_config.enabled = false;
    move_profile->joint_constraint_config.enabled = true;

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
  auto joint_group = env_->getJointGroup(manip.manipulator);
  std::vector<std::string> joint_names = joint_group->getJointNames();

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);

  // These specify the series of points to be optimized
  for (int ind = 0; ind < NUM_STEPS; ind++)
  {
    // Specify a Joint Waypoint as the finish
    JointWaypoint wp{ joint_names, Eigen::VectorXd::Zero(7) };
    wp.getPosition() << 0, 0, 0, -1.57 + ind * 0.1, 0, 0, 0;
    MoveInstruction plan_f(wp, MoveInstructionType::FREESPACE, "TEST_PROFILE");
    program.push_back(plan_f);
  }

  // Create a seed
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  move_profile->cartesian_cost_config.enabled = false;
  move_profile->cartesian_constraint_config.enabled = true;
  move_profile->joint_cost_config.enabled = false;
  move_profile->joint_constraint_config.enabled = true;
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  auto solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", move_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", solver_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
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
  auto joint_group = env_->getJointGroup(manip.manipulator);
  const std::vector<std::string>& joint_names = joint_group->getJointNames();

  // Create a program
  CompositeInstruction program("TEST_PROFILE");
  program.setManipulatorInfo(manip);

  // These specify the series of points to be optimized
  for (int ind = 0; ind < NUM_STEPS; ind++)
  {
    // Specify a Joint Waypoint as the finish
    JointWaypoint wp{ joint_names, Eigen::VectorXd::Zero(7) };
    wp.getPosition() << 0, 0, 0, -1.57 + ind * 0.1, 0, 0, 0;
    MoveInstruction plan_f(wp, MoveInstructionType::FREESPACE, "TEST_PROFILE");
    program.push_back(plan_f);
  }

  // Create a seed
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  move_profile->cartesian_cost_config.enabled = true;
  move_profile->cartesian_constraint_config.enabled = false;
  move_profile->joint_cost_config.enabled = true;
  move_profile->joint_constraint_config.enabled = false;

  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  auto solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();

  // Profile Dictionary
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", move_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile);
  profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", solver_profile);

  // Create Planner
  TrajOptMotionPlanner test_planner(TRAJOPT_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
