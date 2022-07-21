#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_process_managers/task_generators/fix_state_collision_task_generator.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

#include "freespace_example_program.h"

using namespace tesseract_kinematics;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_planning;
using namespace tesseract_collision;

class FixStateCollisionTaskGeneratorUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  ManipulatorInfo manip_;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract_common::TesseractSupportResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();

    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/boxbot.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/boxbot.srdf");
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;

    manip_.manipulator = "manipulator";
  }
};

TEST_F(FixStateCollisionTaskGeneratorUnit, StateInCollisionTest)  // NOLINT
{
  CompositeInstruction program = freespaceExampleProgramABB();
  const InstructionPoly program_instruction{ program };
  InstructionPoly seed = generateSkeletonSeed(program);
  TaskInput input(env_, &program_instruction, manip_, &seed, false, nullptr);

  FixStateCollisionProfile profile;

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
  tesseract_collision::ContactResultMap contacts;
  EXPECT_TRUE(stateInCollision(state, input, profile, contacts));
  EXPECT_FALSE(contacts.empty());
  state[0] = 1.5;

  contacts.clear();
  EXPECT_FALSE(stateInCollision(state, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());
  state[0] = 0.0;
  state[1] = 1.5;

  contacts.clear();
  EXPECT_FALSE(stateInCollision(state, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());

  // Check that the safety margin is obeyed
  profile.collision_check_config.contact_manager_config = tesseract_collision::ContactManagerConfig(0.1);
  state[0] = 0.0;
  state[1] = 1.05;
  contacts.clear();
  EXPECT_TRUE(stateInCollision(state, input, profile, contacts));
  EXPECT_FALSE(contacts.empty());

  profile.collision_check_config.contact_manager_config = tesseract_collision::ContactManagerConfig(0.01);
  contacts.clear();
  EXPECT_FALSE(stateInCollision(state, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());
}

TEST_F(FixStateCollisionTaskGeneratorUnit, WaypointInCollisionTest)  // NOLINT
{
  CompositeInstruction program = freespaceExampleProgramABB();
  const InstructionPoly program_instruction{ program };
  InstructionPoly seed = generateSkeletonSeed(program);
  TaskInput input(env_, &program_instruction, manip_, &seed, false, nullptr);

  FixStateCollisionProfile profile;

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
  JointWaypointPoly waypoint{ JointWaypoint({ "boxbot_x_joint", "boxbot_y_joint" }, state) };
  tesseract_collision::ContactResultMap contacts;

  EXPECT_TRUE(waypointInCollision(waypoint, input, profile, contacts));
  EXPECT_FALSE(contacts.empty());

  waypoint.getPosition()[0] = 1.5;
  contacts.clear();
  EXPECT_FALSE(waypointInCollision(waypoint, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());

  waypoint.getPosition()[0] = 0.0;
  waypoint.getPosition()[1] = 1.5;
  contacts.clear();
  EXPECT_FALSE(waypointInCollision(waypoint, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());

  // Check that the safety margin is obeyed
  profile.collision_check_config.contact_manager_config = tesseract_collision::ContactManagerConfig(0.1);
  waypoint.getPosition()[0] = 0.0;
  waypoint.getPosition()[1] = 1.05;
  contacts.clear();
  EXPECT_TRUE(waypointInCollision(waypoint, input, profile, contacts));
  EXPECT_FALSE(contacts.empty());

  profile.collision_check_config.contact_manager_config = tesseract_collision::ContactManagerConfig(0.01);
  contacts.clear();
  EXPECT_FALSE(waypointInCollision(waypoint, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());

  // Check that it catches invalid inputs correctly
  CartesianWaypoint cart_wp(Eigen::Isometry3d::Identity());
  contacts.clear();
  EXPECT_FALSE(waypointInCollision(cart_wp, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());
}

TEST_F(FixStateCollisionTaskGeneratorUnit, MoveWaypointFromCollisionRandomSamplerTest)  // NOLINT
{
  CompositeInstruction program = freespaceExampleProgramABB();
  const InstructionPoly program_instruction{ program };
  InstructionPoly seed = generateSkeletonSeed(program);
  TaskInput input(env_, &program_instruction, manip_, &seed, false, nullptr);

  FixStateCollisionProfile profile;

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
  JointWaypointPoly waypoint{ JointWaypoint({ "boxbot_x_joint", "boxbot_y_joint" }, state) };

  // Check that the safety margin is obeyed
  profile.collision_check_config.contact_manager_config = tesseract_collision::ContactManagerConfig(0.1);
  profile.jiggle_factor = 1.0;
  waypoint.getPosition()[0] = 0.0;
  waypoint.getPosition()[1] = 1.09;
  WaypointPoly wp(waypoint);
  tesseract_collision::ContactResultMap contacts;

  // Attempts are 0, so it should still be in collision
  profile.sampling_attempts = 0;
  EXPECT_TRUE(waypointInCollision(wp, input, profile, contacts));
  EXPECT_FALSE(moveWaypointFromCollisionRandomSampler(wp, input, profile));
  EXPECT_TRUE(waypointInCollision(wp, input, profile, contacts));

  // It is very unlikely that this will still fail
  profile.sampling_attempts = 1000;
  EXPECT_TRUE(moveWaypointFromCollisionRandomSampler(wp, input, profile));
  EXPECT_FALSE(waypointInCollision(wp, input, profile, contacts));
}

TEST_F(FixStateCollisionTaskGeneratorUnit, MoveWaypointFromCollisionTrajoptTest)  // NOLINT
{
  CompositeInstruction program = freespaceExampleProgramABB();
  const InstructionPoly program_instruction{ program };
  InstructionPoly seed = generateSkeletonSeed(program);
  TaskInput input(env_, &program_instruction, manip_, &seed, false, nullptr);

  FixStateCollisionProfile profile;

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
  JointWaypointPoly waypoint{ JointWaypoint({ "boxbot_x_joint", "boxbot_y_joint" }, state) };

  // Check that the safety margin is obeyed
  profile.collision_check_config.contact_manager_config = tesseract_collision::ContactManagerConfig(0.1);
  profile.jiggle_factor = 1.0;
  waypoint.getPosition()[0] = 0.0;
  waypoint.getPosition()[1] = 1.09;
  WaypointPoly wp(waypoint);
  tesseract_collision::ContactResultMap contacts;

  EXPECT_TRUE(waypointInCollision(wp, input, profile, contacts));
  EXPECT_TRUE(moveWaypointFromCollisionTrajopt(wp, input, profile));
  EXPECT_FALSE(waypointInCollision(wp, input, profile, contacts));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
