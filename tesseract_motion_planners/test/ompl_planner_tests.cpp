/**
 * @file ompl_planner_tests.cpp
 * @brief This contains unit test for the tesseract descartes planner
 *
 * @author Levi Armstrong, Jonathan Meyer
 * @date September 16, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#include <ompl/util/RandomNumbers.h>
#include <functional>
#include <cmath>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/profile/ompl_planner_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_composite_profile_rvss.h>
#include <tesseract_motion_planners/ompl/profile/ompl_waypoint_profile.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/interface_utils.h>

using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;
using namespace tesseract_geometry;
using namespace tesseract_kinematics;
using namespace tesseract_planning;
using namespace tesseract_planning::profile_ns;

const static long SEED = 1;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

static void addBox(tesseract_environment::Environment& env)
{
  Link link_1("box_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.5, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Box>(0.4, 0.001, 0.4);
  link_1.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_1.collision.push_back(collision);

  Joint joint_1("joint_n1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = link_1.getName();
  joint_1.type = JointType::FIXED;

  env.applyCommand(std::make_shared<AddLinkCommand>(link_1, joint_1));
}

template <typename Configurator>
class OMPLTestFixture : public ::testing::Test
{
public:
  using ::testing::Test::Test;

  OMPLTestFixture()
    : env(std::make_shared<Environment>())
    , manip("manipulator", "base_link", "tool0")
    , configurator(std::make_shared<Configurator>())
  {
    // Load scene and srdf
    auto locator = std::make_shared<tesseract_common::SimpleResourceLocator>(locateResource);
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(this->env->init(urdf_path, srdf_path, locator));

    // Add box to environment
    addBox(*env);
  }

  std::shared_ptr<OMPLCompositeProfileRVSS> createCompositeProfile()
  {
    auto composite_profile = std::make_shared<OMPLCompositeProfileRVSS>();
    composite_profile->collision_check_config.contact_manager_config.margin_data_override_type =
        tesseract_collision::CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN;
    composite_profile->collision_check_config.contact_manager_config.margin_data.setDefaultCollisionMargin(0.025);
    composite_profile->collision_check_config.longest_valid_segment_length = 0.1;
    composite_profile->collision_check_config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
    composite_profile->rng_seed = SEED;

    return composite_profile;
  }

  std::shared_ptr<OMPLPlannerProfile> createPlannerProfile()
  {
    auto planner_profile = std::make_shared<OMPLPlannerProfile>();
    planner_profile->params.planning_time = 5.0;
    planner_profile->params.optimize = false;
    planner_profile->params.max_solutions = 2;
    planner_profile->params.simplify = false;
    planner_profile->params.planners = { this->configurator, this->configurator };
    return planner_profile;
  }

  /** @brief Motion planning environment */
  Environment::Ptr env;
  /** @brief Manipulator information for planning */
  const ManipulatorInfo manip;
  /** @brief OMPL planner factory */
  const std::shared_ptr<Configurator> configurator;
  /** @brief Tesseract OMPL motion planner */
  const OMPLMotionPlanner ompl_planner;
  /** @brief Arbitrary trajectory start state */
  const std::vector<double> start_state_ = { -0.6, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  /** @brief Arbitrary trajectory end state */
  const std::vector<double> end_state_ = { 0.6, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  /** @brief Motion planning profile name */
  const std::string profile_name_ = "TEST_PROFILE";
  /** @brief Number of steps in the seed trajectory, equivalent to the the number of states minus one */
  const int seed_steps_ = 10;
};

using Implementations = ::testing::Types<tesseract_planning::SBLConfigurator,
                                         tesseract_planning::PRMConfigurator,
                                         tesseract_planning::PRMstarConfigurator,
                                         tesseract_planning::LazyPRMstarConfigurator,
                                         tesseract_planning::ESTConfigurator,
                                         tesseract_planning::BKPIECE1Configurator,
                                         tesseract_planning::KPIECE1Configurator,
                                         // tesseract_planning::LBKPIECE1Configurator,
                                         // tesseract_planning::RRTConfigurator,
                                         // tesseract_planning::RRTstarConfigurator,
                                         // tesseract_planning::SPARSConfigurator,
                                         // tesseract_planning::TRRTConfigurator,
                                         tesseract_planning::RRTConnectConfigurator>;

TYPED_TEST_CASE(OMPLTestFixture, Implementations);  // NOLINT

TYPED_TEST(OMPLTestFixture, JointStartJointGoal)  // NOLINT
{
  const auto joint_group = this->env->getJointGroup(this->manip.manipulator);
  const auto cur_state = this->env->getState();

  // Specify a start waypoint
  const JointWaypoint wp1(
      joint_group->getJointNames(),
      Eigen::Map<const Eigen::VectorXd>(this->start_state_.data(), static_cast<long>(this->start_state_.size())));

  // Specify a end waypoint
  const JointWaypoint wp2(
      joint_group->getJointNames(),
      Eigen::Map<const Eigen::VectorXd>(this->end_state_.data(), static_cast<long>(this->end_state_.size())));

  // Define Start Instruction
  const PlanInstruction start_instruction(wp1, PlanInstructionType::START, this->profile_name_, this->manip);

  // Define Plan Instructions
  const PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, this->profile_name_, this->manip);
  const PlanInstruction plan_f2(wp1, PlanInstructionType::FREESPACE, this->profile_name_, this->manip);

  // Create a program
  CompositeInstruction program;
  program.setProfile(this->profile_name_);
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(this->manip);
  program.push_back(plan_f1);
  program.push_back(plan_f2);

  // Create Planner Request
  PlannerRequest request;
  request.instructions = program;
  request.seed = generateSeed(program, cur_state, this->env, 3.14, 1.0, 3.14, this->seed_steps_);
  request.env = this->env;
  request.env_state = cur_state;

  // Add the profiles
  {
    auto d = std::make_unique<ProfileDictionary>();
    d->planner_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = this->createPlannerProfile();
    d->composite_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = this->createCompositeProfile();
    d->waypoint_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = std::make_shared<OMPLWaypointProfile>();
    request.profiles = std::move(d);
  }

  PlannerResponse planner_response;
  auto status = this->ompl_planner.solve(request, planner_response);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("CI Error: %s", status.message().c_str());
  }

  ASSERT_TRUE(&status);
  EXPECT_TRUE(planner_response.results.hasStartInstruction());
  EXPECT_GE(getMoveInstructionCount(planner_response.results), 2 * this->seed_steps_ + 1);
  EXPECT_EQ(planner_response.results.size(), 2);
  EXPECT_TRUE(wp1.isApprox(getJointPosition(getFirstMoveInstruction(planner_response.results)->getWaypoint()), 1e-5));
  EXPECT_TRUE(wp2.isApprox(
      getJointPosition(
          getLastMoveInstruction(planner_response.results.front().as<CompositeInstruction>())->getWaypoint()),
      1e-5));
}

TYPED_TEST(OMPLTestFixture, StartStateInCollision)
{
  const auto joint_group = this->env->getJointGroup(this->manip.manipulator);
  const auto cur_state = this->env->getState();

  // Check for start state in collision error
  const std::vector<double> swp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };
  const JointWaypoint wp1(joint_group->getJointNames(),
                          Eigen::Map<const Eigen::VectorXd>(swp.data(), static_cast<long>(swp.size())));
  const PlanInstruction start_instruction(wp1, PlanInstructionType::START, this->profile_name_, this->manip);

  // Specify a end waypoint
  const JointWaypoint wp2(
      joint_group->getJointNames(),
      Eigen::Map<const Eigen::VectorXd>(this->end_state_.data(), static_cast<long>(this->end_state_.size())));
  const PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, this->profile_name_, this->manip);

  // Create a new program
  CompositeInstruction program;
  program.setProfile(this->profile_name_);
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(this->manip);
  program.push_back(plan_f1);

  // Update Configuration
  PlannerRequest request;
  request.instructions = program;
  request.seed = generateSeed(program, cur_state, this->env, 3.14, 1.0, 3.14, this->seed_steps_);
  request.env = this->env;
  request.env_state = cur_state;

  // Add the profiles
  {
    auto d = std::make_unique<ProfileDictionary>();
    d->planner_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = this->createPlannerProfile();
    d->composite_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = this->createCompositeProfile();
    d->waypoint_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = std::make_shared<OMPLWaypointProfile>();
    request.profiles = std::move(d);
  }

  // Solve
  PlannerResponse planner_response;
  auto status = this->ompl_planner.solve(request, planner_response);
  EXPECT_FALSE(status);
}

TYPED_TEST(OMPLTestFixture, EndStateInCollision)
{
  const auto joint_group = this->env->getJointGroup(this->manip.manipulator);
  const auto cur_state = this->env->getState();

  // Specify a start waypoint
  const JointWaypoint wp1(
      joint_group->getJointNames(),
      Eigen::Map<const Eigen::VectorXd>(this->start_state_.data(), static_cast<long>(this->start_state_.size())));
  const PlanInstruction start_instruction(wp1, PlanInstructionType::START, this->profile_name_, this->manip);

  // Check for end state in collision error
  const std::vector<double> ewp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };
  const JointWaypoint wp2(joint_group->getJointNames(),
                          Eigen::Map<const Eigen::VectorXd>(ewp.data(), static_cast<long>(ewp.size())));
  const PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, this->profile_name_, this->manip);

  // Create a new program
  CompositeInstruction program;
  program.setProfile(this->profile_name_);
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(this->manip);
  program.push_back(plan_f1);

  PlannerRequest request;
  request.seed = generateSeed(program, cur_state, this->env, 3.14, 1.0, 3.14, this->seed_steps_);
  request.instructions = program;
  request.env = this->env;
  request.env_state = cur_state;

  // Add the profiles
  {
    auto d = std::make_unique<ProfileDictionary>();
    d->planner_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = this->createPlannerProfile();
    d->composite_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = this->createCompositeProfile();
    d->waypoint_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = std::make_shared<OMPLWaypointProfile>();
    request.profiles = std::move(d);
  }

  // Set new configuration and solve
  PlannerResponse planner_response;
  auto status = this->ompl_planner.solve(request, planner_response);
  EXPECT_FALSE(status);
}

TYPED_TEST(OMPLTestFixture, JointStartCartesianGoal)
{
  const auto kin_group = this->env->getKinematicGroup(this->manip.manipulator);
  const auto cur_state = this->env->getState();

  // Specify a start waypoint
  const JointWaypoint wp1(
      kin_group->getJointNames(),
      Eigen::Map<const Eigen::VectorXd>(this->start_state_.data(), static_cast<long>(this->start_state_.size())));

  // Specify a end waypoint
  const auto goal_jv =
      Eigen::Map<const Eigen::VectorXd>(this->end_state_.data(), static_cast<long>(this->end_state_.size()));
  const Eigen::Isometry3d goal = kin_group->calcFwdKin(goal_jv).at(this->manip.tcp_frame);
  const CartesianWaypoint wp2 = goal;

  // Define Start Instruction
  const PlanInstruction start_instruction(wp1, PlanInstructionType::START, this->profile_name_, this->manip);

  // Define Plan Instructions
  const PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, this->profile_name_, this->manip);
  const PlanInstruction plan_f2(wp1, PlanInstructionType::FREESPACE, this->profile_name_, this->manip);

  // Create a program
  CompositeInstruction program;
  program.setProfile(this->profile_name_);
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(this->manip);
  program.push_back(plan_f1);
  program.push_back(plan_f2);

  // Create Planner Request
  PlannerRequest request;
  request.instructions = program;
  request.seed = generateSeed(program, cur_state, this->env, 3.14, 1.0, 3.14, this->seed_steps_);
  request.env = this->env;
  request.env_state = cur_state;

  // Add the profiles
  {
    auto d = std::make_unique<ProfileDictionary>();
    d->planner_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = this->createPlannerProfile();
    d->composite_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = this->createCompositeProfile();
    d->waypoint_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = std::make_shared<OMPLWaypointProfile>();
    request.profiles = std::move(d);
  }

  PlannerResponse planner_response;
  auto status = this->ompl_planner.solve(request, planner_response);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("CI Error: %s", status.message().c_str());
  }
  ASSERT_TRUE(&status);
  ASSERT_TRUE(planner_response.results.hasStartInstruction());
  EXPECT_GE(getMoveInstructionCount(planner_response.results), 2 * this->seed_steps_ + 1);
  EXPECT_TRUE(wp1.isApprox(getJointPosition(getFirstMoveInstruction(planner_response.results)->getWaypoint()), 1e-5));
  EXPECT_TRUE(wp1.isApprox(getJointPosition(getLastMoveInstruction(planner_response.results)->getWaypoint()), 1e-5));

  const MoveInstruction* cart_move =
      getLastMoveInstruction(planner_response.results.front().as<CompositeInstruction>());
  const Eigen::VectorXd& cart_move_joints = getJointPosition(cart_move->getWaypoint());
  const Eigen::Isometry3d check_goal = kin_group->calcFwdKin(cart_move_joints).at(this->manip.tcp_frame);
  EXPECT_TRUE(wp2.isApprox(check_goal, 1e-3));
}

TYPED_TEST(OMPLTestFixture, CartesianStartJointGoal)
{
  const auto kin_group = this->env->getKinematicGroup(this->manip.manipulator);
  const auto cur_state = this->env->getState();

  // Specify a start waypoint
  const auto start_jv =
      Eigen::Map<const Eigen::VectorXd>(this->start_state_.data(), static_cast<long>(this->start_state_.size()));
  const Eigen::Isometry3d start = kin_group->calcFwdKin(start_jv).at(this->manip.tcp_frame);
  const CartesianWaypoint wp1 = start;

  // Specify a end waypoint
  const JointWaypoint wp2(
      kin_group->getJointNames(),
      Eigen::Map<const Eigen::VectorXd>(this->end_state_.data(), static_cast<long>(this->end_state_.size())));

  // Define Start Instruction
  const PlanInstruction start_instruction(wp1, PlanInstructionType::START, this->profile_name_, this->manip);

  // Define Plan Instructions
  const PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, this->profile_name_, this->manip);
  const PlanInstruction plan_f2(wp1, PlanInstructionType::FREESPACE, this->profile_name_, this->manip);

  // Create a program
  CompositeInstruction program;
  program.setProfile(this->profile_name_);
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(this->manip);
  program.push_back(plan_f1);
  program.push_back(plan_f2);

  // Create Planner Request
  PlannerRequest request;
  request.instructions = program;
  request.seed = generateSeed(program, cur_state, this->env, 3.14, 1.0, 3.14, this->seed_steps_);
  request.env = this->env;
  request.env_state = cur_state;

  // Add the profiles
  {
    auto d = std::make_unique<ProfileDictionary>();
    d->planner_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = this->createPlannerProfile();
    d->composite_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = this->createCompositeProfile();
    d->waypoint_profiles[OMPL_DEFAULT_NAMESPACE][this->profile_name_] = std::make_shared<OMPLWaypointProfile>();
    request.profiles = std::move(d);
  }

  PlannerResponse planner_response;
  auto status = this->ompl_planner.solve(request, planner_response);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("CI Error: %s", status.message().c_str());
  }

  ASSERT_TRUE(&status);
  ASSERT_TRUE(planner_response.results.hasStartInstruction());
  EXPECT_GE(getMoveInstructionCount(planner_response.results), 2 * this->seed_steps_ + 1);

  // Check the start/end Cartesian coordinate against the target waypoint
  auto check_cartesian_pose = [&](const MoveInstruction* mi) {
    const Eigen::VectorXd& joints = getJointPosition(mi->getWaypoint());
    const Eigen::Isometry3d pose = kin_group->calcFwdKin(joints).at(this->manip.tcp_frame);
    EXPECT_TRUE(wp1.isApprox(pose, 1e-3));
  };
  check_cartesian_pose(getFirstMoveInstruction(planner_response.results));
  check_cartesian_pose(getLastMoveInstruction(planner_response.results));

  // Check the joint move
  const MoveInstruction* last_mi = getLastMoveInstruction(planner_response.results.front().as<CompositeInstruction>());
  EXPECT_TRUE(wp2.isApprox(getJointPosition(last_mi->getWaypoint()), 1e-5));
}

// TEST(OMPLMultiPlanner, OMPLMultiPlannerUnit)  // NOLINT
//{
//  EXPECT_EQ(ompl::RNG::getSeed(), SEED) << "Randomization seed does not match expected: " << ompl::RNG::getSeed()
//                                        << " vs. " << SEED;

//  // Step 1: Load scene and srdf
//  tesseract_scene_graph::ResourceLocator::Ptr locator =
//      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
//  Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
//  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
//  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
//  EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));

//  // Step 2: Add box to environment
//  addBox(*(env));

//  // Step 3: Create ompl planner config and populate it
//  auto kin = this->env->getManipulatorManager()->getFwdKinematicSolver(manip.manipulator);
//  std::vector<double> swp = start_state;
//  std::vector<double> ewp = end_state;

//  OMPLMotionPlanner ompl_planner;

//  std::vector<OMPLPlannerConfigurator::ConstPtr> planners = { std::make_shared<SBLConfigurator>(),
//                                                              std::make_shared<RRTConnectConfigurator>() };
//  auto ompl_config = std::make_shared<OMPLPlannerFreespaceConfig>(tesseract, manip.manipulator, planners);

//  ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp,
//  kin->getJointNames()); ompl_config->end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp,
//  kin->getJointNames()); ompl_config->collision_safety_margin = 0.02; ompl_config->planning_time = 5.0;
//  ompl_config->max_solutions = 2;
//  ompl_config->longest_valid_segment_fraction = 0.01;

//  ompl_config->collision_continuous = true;
//  ompl_config->collision_check = true;
//  ompl_config->simplify = false;

//  // Set the planner configuration
//  ompl_planner.setConfiguration(ompl_config);

//  tesseract_motion_planners::PlannerResponse ompl_planning_response;
//  tesseract_common::StatusCode status = ompl_planner.solve(ompl_planning_response);

//  if (!status)
//  {
//    CONSOLE_BRIDGE_logError("CI Error: %s", status.message().c_str());
//  }
//  EXPECT_TRUE(&status);
//  EXPECT_GE(ompl_planning_response.joint_trajectory.trajectory.rows(), request.seed.size());

//  // Check for start state in collision error
//  swp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };
//  ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp,
//  kin->getJointNames());

//  ompl_planner.setConfiguration(ompl_config);
//  status = ompl_planner.solve(ompl_planning_response);

//  EXPECT_FALSE(status);

//  // Check for start state in collision error
//  swp = start_state;
//  ewp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };
//  ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp,
//  kin->getJointNames()); ompl_config->end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp,
//  kin->getJointNames());

//  ompl_planner.setConfiguration(ompl_config);
//  status = ompl_planner.solve(ompl_planning_response);

//  EXPECT_FALSE(status);

//  // Reset start and end waypoints
//  swp = start_state;
//  ewp = end_state;
//  ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp,
//  kin->getJointNames()); ompl_config->end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp,
//  kin->getJointNames());
//}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ompl::RNG::setSeed(static_cast<unsigned long>(SEED));

  return RUN_ALL_TESTS();
}
