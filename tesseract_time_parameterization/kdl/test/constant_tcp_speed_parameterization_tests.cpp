/**
 * @copyright Copyright (c) 2025, Southwest Research Institute
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/common/unit_test_utils.h>
#include <tesseract/environment/environment.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_time_parameterization/kdl/cereal_serialization.h>
#include <tesseract_time_parameterization/kdl/constant_tcp_speed_parameterization.h>
#include <tesseract_time_parameterization/kdl/constant_tcp_speed_parameterization_profiles.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

using namespace tesseract::time_parameterization;
using namespace tesseract::command_language;

// Initialize one-joint, straight-line trajectory
CompositeInstruction createStraightTrajectory()
{
  const int num = 10;
  const double max = 2.0;

  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  CompositeInstruction program;
  for (int i = 0; i < num; i++)
  {
    StateWaypoint swp{ joint_names, Eigen::VectorXd::Zero(6) };
    swp.getPosition()[0] = i * max / num;
    program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  // leave final velocity/acceleration unset
  StateWaypoint swp{ joint_names, Eigen::VectorXd::Zero(6) };
  swp.getPosition()[0] = max;
  program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));

  return program;
}

bool checkForVelocityAndAcceleration(const CompositeInstruction& program)
{
  for (const auto& i : program)
  {
    const auto& swp = i.as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
    if (swp.getVelocity().size() == 0 || swp.getAcceleration().size() == 0)
      return false;
  }

  return true;
}

class ConstantTCPSpeedParameterizationUnit : public ::testing::Test
{
protected:
  std::string name_{ "ConstantTCPSpeedParameterizationUnit" };
  tesseract::common::GeneralResourceLocator::Ptr locator_;
  tesseract::environment::Environment::Ptr env_;
  tesseract::common::ManipulatorInfo manip_;

  void SetUp() override
  {
    locator_ = std::make_shared<tesseract::common::GeneralResourceLocator>();
    auto env = std::make_shared<tesseract::environment::Environment>();

    std::filesystem::path urdf_path(
        locator_->locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")->getFilePath());
    std::filesystem::path srdf_path(
        locator_->locateResource("package://tesseract/support/urdf/abb_irb2400.srdf")->getFilePath());
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator_));
    env_ = env;

    manip_.manipulator = "manipulator";
    manip_.working_frame = "base_link";
    manip_.tcp_frame = "tool0";
  }
};

TEST_F(ConstantTCPSpeedParameterizationUnit, profileConstructor)  // NOLINT
{
  ConstantTCPSpeedParameterizationCompositeProfile profile(3, 5, 7, 9, 11, 13);
  EXPECT_NEAR(profile.max_translational_velocity, 3, 1e-6);
  EXPECT_NEAR(profile.max_rotational_velocity, 5, 1e-6);
  EXPECT_NEAR(profile.max_translational_acceleration, 7, 1e-6);
  EXPECT_NEAR(profile.max_rotational_acceleration, 9, 1e-6);
  EXPECT_NEAR(profile.max_velocity_scaling_factor, 11, 1e-6);
  EXPECT_NEAR(profile.max_acceleration_scaling_factor, 13, 1e-6);
}

TEST_F(ConstantTCPSpeedParameterizationUnit, ConstantTCPSpeedParameterizationTest)  // NOLINT
{
  CompositeInstruction program = createStraightTrajectory();
  program.setManipulatorInfo(manip_);

  {
    // Profile
    auto profile = std::make_shared<ConstantTCPSpeedParameterizationCompositeProfile>();
    profile->max_translational_velocity = 0.5;
    profile->max_rotational_velocity = 0.5;
    profile->max_translational_acceleration = 0.5;
    profile->max_rotational_acceleration = 0.5;
    profile->max_velocity_scaling_factor = 1.0;
    profile->max_acceleration_scaling_factor = 1.0;

    // Serialization
    tesseract::common::testSerializationDerivedClass<tesseract::common::Profile,
                                                     ConstantTCPSpeedParameterizationCompositeProfile>(profile,
                                                                                                       "ConstantTCPSpee"
                                                                                                       "d"
                                                                                                       "Parameterizatio"
                                                                                                       "n"
                                                                                                       "Test");

    // Profile Dictionary
    tesseract::common::ProfileDictionary profiles;
    profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);

    // Solve
    ConstantTCPSpeedParameterization time_parameterization(name_);
    EXPECT_EQ(time_parameterization.getName(), name_);
    EXPECT_TRUE(time_parameterization.compute(program, *env_, profiles));
    EXPECT_TRUE(checkForVelocityAndAcceleration(program));
    ASSERT_LT(program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(), 5.001);

    // Error
    std::string empty_name;
    EXPECT_ANY_THROW(ConstantTCPSpeedParameterization(std::move(empty_name)));  // NOLINT
  }

  {
    // Profile
    auto profile = std::make_shared<ConstantTCPSpeedParameterizationCompositeProfile>();
    profile->max_translational_velocity = 1.0;
    profile->max_rotational_velocity = 0.5;
    profile->max_translational_acceleration = 0.5;
    profile->max_rotational_acceleration = 0.5;
    profile->max_velocity_scaling_factor = 1.0;
    profile->max_acceleration_scaling_factor = 1.0;

    // Profile Dictionary
    tesseract::common::ProfileDictionary profiles;
    profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);

    // Solve
    ConstantTCPSpeedParameterization time_parameterization(name_);
    EXPECT_TRUE(time_parameterization.compute(program, *env_, profiles));
    EXPECT_TRUE(checkForVelocityAndAcceleration(program));
    ASSERT_LT(program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(), 4.001);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  return RUN_ALL_TESTS();
}
