#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_environment/environment.h>
#include <tesseract_task_composer/planning/profiles/fix_state_bounds_profile.h>
#include <tesseract_task_composer/planning/nodes/fix_state_bounds_task.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

using namespace tesseract_planning;
using namespace tesseract_environment;
using tesseract_common::ManipulatorInfo;

static const std::string FIX_STATE_BOUNDS_TASK_NAME = "FixStateBoundsTask";

class FixStateBoundsTaskUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  ManipulatorInfo manip_;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();

    std::filesystem::path urdf_path(
        locator->locateResource("package://tesseract_support/urdf/abb_irb2400.urdf")->getFilePath());
    std::filesystem::path srdf_path(
        locator->locateResource("package://tesseract_support/urdf/abb_irb2400.srdf")->getFilePath());
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;

    manip_.manipulator = "manipulator";
  }
};

CompositeInstruction createProgram(const Eigen::VectorXd& start_state,
                                   const Eigen::VectorXd& goal_state,
                                   const std::string& composite_profile)
{
  CompositeInstruction program(composite_profile, ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  JointWaypoint wp1{ joint_names, start_state };
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE);
  start_instruction.setDescription("Start Instruction");
  program.push_back(start_instruction);

  JointWaypoint wp2{ joint_names, start_state + ((goal_state - start_state) / 2) };
  MoveInstruction plan_f0(wp2, MoveInstructionType::FREESPACE);
  program.push_back(plan_f0);

  JointWaypoint wp3{ joint_names, goal_state };
  MoveInstruction plan_f1(wp3, MoveInstructionType::FREESPACE);
  program.push_back(plan_f1);

  return program;
}

void checkProgram(const Environment::Ptr& env,
                  const ManipulatorInfo& manip,
                  const Eigen::VectorXd& start_state,
                  const Eigen::VectorXd& goal_state,
                  std::shared_ptr<tesseract_common::ProfileDictionary>& profiles,
                  FixStateBoundsProfile::Settings setting,
                  bool pre_check_return,
                  int expected_return)
{
  auto joint_limits = env->getJointGroup(manip.manipulator)->getLimits().joint_limits;

  CompositeInstruction program = createProgram(start_state, goal_state, DEFAULT_PROFILE_KEY);

  // Create data storage
  auto task_data = std::make_unique<TaskComposerDataStorage>();
  task_data->setData("input_program", program);
  task_data->setData("environment", std::shared_ptr<const Environment>(env));
  task_data->setData("profiles", profiles);

  // Create context
  auto task_context = std::make_shared<TaskComposerContext>("fix_state_bounds_unit", std::move(task_data));

  // Create task
  FixStateBoundsTask task(FIX_STATE_BOUNDS_TASK_NAME, "input_program", "environment", "profiles", "output_program");

  // Manual Check of program
  auto flattened = program.flatten(moveFilter);
  bool inside_limits = true;
  for (const auto& instruction : flattened)
    inside_limits &= isWithinJointLimits(instruction.get().as<MoveInstructionPoly>().getWaypoint(), joint_limits);
  EXPECT_EQ(inside_limits, pre_check_return);

  EXPECT_EQ(task.run(*task_context), expected_return);

  if (expected_return == 1)
  {
    auto task_program = task_context->data_storage->getData("output_program").as<CompositeInstruction>();
    auto task_flattened = task_program.flatten(moveFilter);

    switch (setting)
    {
      case FixStateBoundsProfile::Settings::START_ONLY:
      {
        EXPECT_TRUE(
            isWithinJointLimits(task_flattened.front().get().as<MoveInstructionPoly>().getWaypoint(), joint_limits));
        break;
      }
      case FixStateBoundsProfile::Settings::END_ONLY:
      {
        EXPECT_TRUE(
            isWithinJointLimits(task_flattened.back().get().as<MoveInstructionPoly>().getWaypoint(), joint_limits));
        break;
      }
      case FixStateBoundsProfile::Settings::ALL:
      {
        bool inside_limits = true;
        for (const auto& instruction : task_flattened)
          inside_limits &= isWithinJointLimits(instruction.get().as<MoveInstructionPoly>().getWaypoint(), joint_limits);
        EXPECT_TRUE(inside_limits);
        break;
      }
      case FixStateBoundsProfile::Settings::DISABLED:
        break;
    }
  }
}

TEST_F(FixStateBoundsTaskUnit, stateInBounds)  // NOLINT
{
  using Settings = FixStateBoundsProfile::Settings;
  auto joint_limits = env_->getJointGroup(manip_.manipulator)->getLimits().joint_limits;

  auto state_bounds_profile = std::make_shared<FixStateBoundsProfile>();
  state_bounds_profile->max_deviation_global = 0.2;
  state_bounds_profile->lower_bounds_reduction = 1e-3;
  state_bounds_profile->upper_bounds_reduction = 1e-3;

  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  profiles->addProfile(FIX_STATE_BOUNDS_TASK_NAME, DEFAULT_PROFILE_KEY, state_bounds_profile);

  Eigen::VectorXd mid_state = joint_limits.col(0) + (joint_limits.col(1) - joint_limits.col(0)) / 2.;
  {  // All are valid
    // Check using ALL
    state_bounds_profile->mode = Settings::ALL;
    checkProgram(env_, manip_, mid_state, mid_state, profiles, Settings::ALL, true, 1);

    // Check using END_ONLY
    state_bounds_profile->mode = Settings::END_ONLY;
    checkProgram(env_, manip_, mid_state, mid_state, profiles, Settings::END_ONLY, true, 1);

    // Check using START_ONLY
    state_bounds_profile->mode = Settings::START_ONLY;
    checkProgram(env_, manip_, mid_state, mid_state, profiles, Settings::START_ONLY, true, 1);

    // Check using DISABLED
    state_bounds_profile->mode = Settings::DISABLED;
    checkProgram(env_, manip_, mid_state, mid_state, profiles, Settings::DISABLED, true, 1);
  }

  {  // Last is outside the upper limits but inside the max deviation threshold
    Eigen::VectorXd invalid_state = joint_limits.col(1).array() + 1.e-5;

    // Check using ALL
    state_bounds_profile->mode = Settings::ALL;
    checkProgram(env_, manip_, mid_state, invalid_state, profiles, Settings::ALL, false, 1);

    // Check using END_ONLY
    state_bounds_profile->mode = Settings::END_ONLY;
    checkProgram(env_, manip_, mid_state, invalid_state, profiles, Settings::END_ONLY, false, 1);

    // Check using START_ONLY
    state_bounds_profile->mode = Settings::START_ONLY;
    checkProgram(env_, manip_, mid_state, invalid_state, profiles, Settings::START_ONLY, false, 1);

    // Check using DISABLED
    state_bounds_profile->mode = Settings::DISABLED;
    checkProgram(env_, manip_, mid_state, invalid_state, profiles, Settings::DISABLED, false, 1);
  }

  {  // First is outside the upper limits but inside the max deviation threshold
    Eigen::VectorXd invalid_state = joint_limits.col(0).array() - 1.e-5;

    // Check using ALL
    state_bounds_profile->mode = Settings::ALL;
    checkProgram(env_, manip_, invalid_state, mid_state, profiles, Settings::ALL, false, 1);

    // Check using END_ONLY
    state_bounds_profile->mode = Settings::END_ONLY;
    checkProgram(env_, manip_, invalid_state, mid_state, profiles, Settings::END_ONLY, false, 1);

    // Check using START_ONLY
    state_bounds_profile->mode = Settings::START_ONLY;
    checkProgram(env_, manip_, invalid_state, mid_state, profiles, Settings::START_ONLY, false, 1);

    // Check using DISABLED
    state_bounds_profile->mode = Settings::DISABLED;
    checkProgram(env_, manip_, invalid_state, mid_state, profiles, Settings::DISABLED, false, 1);
  }

  {  // Last is outside the upper limits and outside the max deviation threshold
    Eigen::VectorXd invalid_state = joint_limits.col(1).array() + .3;

    // Check using ALL
    state_bounds_profile->mode = Settings::ALL;
    checkProgram(env_, manip_, mid_state, invalid_state, profiles, Settings::ALL, false, 0);

    // Check using END_ONLY
    state_bounds_profile->mode = Settings::END_ONLY;
    checkProgram(env_, manip_, mid_state, invalid_state, profiles, Settings::END_ONLY, false, 0);

    // Check using START_ONLY
    state_bounds_profile->mode = Settings::START_ONLY;
    checkProgram(env_, manip_, mid_state, invalid_state, profiles, Settings::START_ONLY, false, 1);

    // Check using DISABLED
    state_bounds_profile->mode = Settings::DISABLED;
    checkProgram(env_, manip_, mid_state, invalid_state, profiles, Settings::DISABLED, false, 1);
  }

  {  // First is outside the upper limits and outside the max deviation threshold
    Eigen::VectorXd invalid_state = joint_limits.col(0).array() - .3;

    // Check using ALL
    state_bounds_profile->mode = Settings::ALL;
    checkProgram(env_, manip_, invalid_state, mid_state, profiles, Settings::ALL, false, 0);

    // Check using END_ONLY
    state_bounds_profile->mode = Settings::END_ONLY;
    checkProgram(env_, manip_, invalid_state, mid_state, profiles, Settings::END_ONLY, false, 1);

    // Check using START_ONLY
    state_bounds_profile->mode = Settings::START_ONLY;
    checkProgram(env_, manip_, invalid_state, mid_state, profiles, Settings::START_ONLY, false, 0);

    // Check using DISABLED
    state_bounds_profile->mode = Settings::DISABLED;
    checkProgram(env_, manip_, invalid_state, mid_state, profiles, Settings::DISABLED, false, 1);
  }

  {  // All are outside the upper limits and outside the max deviation threshold
    Eigen::VectorXd start_state = joint_limits.col(1).array() + .3;
    Eigen::VectorXd end_state = joint_limits.col(1).array() + .6;

    // Check using ALL
    state_bounds_profile->mode = Settings::ALL;
    checkProgram(env_, manip_, start_state, end_state, profiles, Settings::ALL, false, 0);

    // Check using END_ONLY
    state_bounds_profile->mode = Settings::END_ONLY;
    checkProgram(env_, manip_, start_state, end_state, profiles, Settings::END_ONLY, false, 0);

    // Check using START_ONLY
    state_bounds_profile->mode = Settings::START_ONLY;
    checkProgram(env_, manip_, start_state, end_state, profiles, Settings::START_ONLY, false, 0);

    // Check using DISABLED
    state_bounds_profile->mode = Settings::DISABLED;
    checkProgram(env_, manip_, start_state, end_state, profiles, Settings::DISABLED, false, 1);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
