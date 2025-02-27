#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/continuous_contact_check_task.h>
#include <tesseract_task_composer/planning/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/planning/nodes/format_as_input_task.h>
#include <tesseract_task_composer/planning/nodes/format_as_result_task.h>
#include <tesseract_task_composer/planning/nodes/format_planning_input_task.h>
#include <tesseract_task_composer/planning/nodes/min_length_task.h>
#include <tesseract_task_composer/planning/nodes/fix_state_bounds_task.h>
#include <tesseract_task_composer/planning/nodes/fix_state_collision_task.h>
#include <tesseract_task_composer/planning/nodes/profile_switch_task.h>
#include <tesseract_task_composer/planning/nodes/update_end_state_task.h>
#include <tesseract_task_composer/planning/nodes/update_start_and_end_state_task.h>
#include <tesseract_task_composer/planning/nodes/update_start_state_task.h>
#include <tesseract_task_composer/planning/nodes/upsample_trajectory_task.h>
#include <tesseract_task_composer/planning/nodes/iterative_spline_parameterization_task.h>
#include <tesseract_task_composer/planning/nodes/time_optimal_parameterization_task.h>
#include <tesseract_task_composer/planning/nodes/ruckig_trajectory_smoothing_task.h>
#include <tesseract_task_composer/planning/nodes/motion_planner_task.hpp>
#include <tesseract_task_composer/planning/nodes/raster_motion_task.h>
#include <tesseract_task_composer/planning/nodes/raster_only_motion_task.h>

#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_log.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/test_suite/task_composer_serialization_utils.hpp>
#include <tesseract_task_composer/core/test_suite/test_programs.hpp>

#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_common/types.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/joint_state.h>

#include <tesseract_environment/environment.h>

using namespace tesseract_planning;

class TesseractTaskComposerPlanningUnit : public ::testing::Test
{
protected:
  tesseract_common::GeneralResourceLocator::Ptr locator_;
  tesseract_environment::Environment::Ptr env_;
  tesseract_common::ManipulatorInfo manip_;

  void SetUp() override
  {
    locator_ = std::make_shared<tesseract_common::GeneralResourceLocator>();
    auto env = std::make_shared<tesseract_environment::Environment>();

    std::filesystem::path urdf_path(
        locator_->locateResource("package://tesseract_support/urdf/abb_irb2400.urdf")->getFilePath());
    std::filesystem::path srdf_path(
        locator_->locateResource("package://tesseract_support/urdf/abb_irb2400.srdf")->getFilePath());
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator_));
    env_ = env;

    manip_.manipulator = "manipulator";
    manip_.working_frame = "base_link";
    manip_.tcp_frame = "tool0";
  }
};

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerContinuousContactCheckTaskTests)  // NOLINT
{
  {  // Construction
    ContinuousContactCheckTask task;
    EXPECT_EQ(task.getName(), "ContinuousContactCheckTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles)";
    YAML::Node config = YAML::Load(str);
    ContinuousContactCheckTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 3);
    EXPECT_EQ(task.getInputKeys().get(ContinuousContactCheckTask::INPUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(ContinuousContactCheckTask::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getInputKeys().get(ContinuousContactCheckTask::INPUT_PROFILES_PORT), "profiles");
    EXPECT_EQ(task.getOutputKeys().size(), 0);
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: [input_data, input_data1])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<ContinuousContactCheckTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<ContinuousContactCheckTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<ContinuousContactCheckTask>("abc", "input_data", "environment", "profiles", true);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerContinuousContactCheckTaskTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context =
        std::make_unique<TaskComposerContext>("TaskComposerContinuousContactCheckTaskTests", std::move(data));
    ContinuousContactCheckTask task(
        "TaskComposerContinuousContactCheckTaskTests", "input_data", "environment", "profiles", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());

    test_suite::runSerializationTest(*node_info, "TaskComposerContinuousContactCheckNodeInfoTests");
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context =
        std::make_unique<TaskComposerContext>("TaskComposerContinuousContactCheckTaskTests", std::move(data));
    ContinuousContactCheckTask task(
        "TaskComposerContinuousContactCheckTaskTests", "input_data", "environment", "profiles", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing environment data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    data->setData("profiles", profiles);
    auto context =
        std::make_unique<TaskComposerContext>("TaskComposerContinuousContactCheckTaskTests", std::move(data));
    ContinuousContactCheckTask task(
        "TaskComposerContinuousContactCheckTaskTests", "input_data", "environment", "profiles", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing profiles data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    auto context =
        std::make_unique<TaskComposerContext>("TaskComposerContinuousContactCheckTaskTests", std::move(data));
    ContinuousContactCheckTask task(
        "TaskComposerContinuousContactCheckTaskTests", "input_data", "environment", "profiles", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure collision
    auto profiles = std::make_shared<ProfileDictionary>();

    auto profile = std::make_unique<ContactCheckProfile>();
    profile->config.contact_manager_config = tesseract_collision::ContactManagerConfig(1.5);
    profile->config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    profiles->addProfile("TaskComposerContinuousContactCheckTaskTests", DEFAULT_PROFILE_KEY, std::move(profile));

    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context =
        std::make_unique<TaskComposerContext>("TaskComposerContinuousContactCheckTaskTests", std::move(data));
    ContinuousContactCheckTask task(
        "TaskComposerContinuousContactCheckTaskTests", "input_data", "environment", "profiles", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(node_info->data_storage.getData("contact_results")
                  .as<std::vector<tesseract_collision::ContactResultMap>>()
                  .empty(),
              false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerDiscreteContactCheckTaskTests)  // NOLINT
{
  {  // Construction
    DiscreteContactCheckTask task;
    EXPECT_EQ(task.getName(), "DiscreteContactCheckTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles)";
    YAML::Node config = YAML::Load(str);
    DiscreteContactCheckTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 3);
    EXPECT_EQ(task.getInputKeys().get(DiscreteContactCheckTask::INPUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(DiscreteContactCheckTask::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getInputKeys().get(DiscreteContactCheckTask::INPUT_PROFILES_PORT), "profiles");
    EXPECT_EQ(task.getOutputKeys().size(), 0);
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: [input_data, input_data1])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<DiscreteContactCheckTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<DiscreteContactCheckTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<DiscreteContactCheckTask>("abc", "input_data", "environment", "profiles", true);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerDiscreteContactCheckTaskTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("TaskComposerDiscreteContactCheckTaskTests", std::move(data));
    DiscreteContactCheckTask task(
        "TaskComposerDiscreteContactCheckTaskTests", "input_data", "environment", "profiles", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());

    test_suite::runSerializationTest(*node_info, "TaskComposerDiscreteContactCheckNodeInfoTests");
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("TaskComposerDiscreteContactCheckTaskTests", std::move(data));
    DiscreteContactCheckTask task(
        "TaskComposerDiscreteContactCheckTaskTests", "input_data", "environment", "profiles", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing environment data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("TaskComposerDiscreteContactCheckTaskTests", std::move(data));
    DiscreteContactCheckTask task(
        "TaskComposerDiscreteContactCheckTaskTests", "input_data", "environment", "profiles", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing profiles dat
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    auto context = std::make_unique<TaskComposerContext>("TaskComposerDiscreteContactCheckTaskTests", std::move(data));
    DiscreteContactCheckTask task(
        "TaskComposerDiscreteContactCheckTaskTests", "input_data", "environment", "profiles", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure collision
    auto profiles = std::make_shared<ProfileDictionary>();

    auto profile = std::make_unique<ContactCheckProfile>();
    profile->config.contact_manager_config = tesseract_collision::ContactManagerConfig(1.5);
    profile->config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    profiles->addProfile("TaskComposerDiscreteContactCheckTaskTests", DEFAULT_PROFILE_KEY, std::move(profile));

    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("TaskComposerDiscreteContactCheckTaskTests", std::move(data));
    DiscreteContactCheckTask task(
        "TaskComposerDiscreteContactCheckTaskTests", "input_data", "environment", "profiles", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(node_info->data_storage.getData("contact_results")
                  .as<std::vector<tesseract_collision::ContactResultMap>>()
                  .empty(),
              false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerFormatAsInputTaskTests)  // NOLINT
{
  {  // Construction
    FormatAsInputTask task;
    EXPECT_EQ(task.getName(), "FormatAsInputTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             pre_planning_program: input_data
                             post_planning_program: output_data
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    FormatAsInputTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 2);
    EXPECT_EQ(task.getInputKeys().get(FormatAsInputTask::INPUT_PRE_PLANNING_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(FormatAsInputTask::INPUT_POST_PLANNING_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(FormatAsInputTask::OUTPUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             pre_planning_program: input_data
                             post_planning_program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             pre_planning_program: input_data
                             post_planning_program: output_data
                           outputs:
                             program: output_data
                             extra: extra)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             pre_planning_program: input_data
                             post_planning_program: output_data
                             extra: extra
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<FormatAsInputTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerFormatAsInputTaskTests");
  }

  {  // Test run method
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("output_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FormatAsInputTask task("abc", "input_data", "output_data", "output_data2", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_EQ(context->data_storage->getData("output_data2"), context->data_storage->getData("input_data"));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method with cartesian in input
    auto data = std::make_unique<TaskComposerDataStorage>();
    auto input_program = test_suite::jointInterpolateExampleProgramABB(true);

    // Modify to have cartesian waypoint
    MoveInstructionPoly last = input_program.back().as<MoveInstructionPoly>();
    const auto& jwp = last.getWaypoint().as<JointWaypointPoly>();
    CartesianWaypoint cwp{ Eigen::Isometry3d::Identity() };
    cwp.setSeed(tesseract_common::JointState(jwp.getNames(), jwp.getPosition()));
    last.getWaypoint() = CartesianWaypointPoly(cwp);
    input_program.back() = last;
    EXPECT_EQ(input_program.size(), 2);

    // Set input data
    data->setData("input_data", input_program);
    data->setData("output_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FormatAsInputTask task("abc", "input_data", "output_data", "output_data2", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_EQ(context->data_storage->getData("output_data2"), context->data_storage->getData("input_data"));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method with unconstraint joint waypoint
    auto data = std::make_unique<TaskComposerDataStorage>();
    auto input_program = test_suite::jointInterpolateExampleProgramABB(true);

    // Modify to have unconstrained joint waypoint
    auto& jwp = input_program.back().as<MoveInstructionPoly>().getWaypoint().as<JointWaypointPoly>();
    jwp.setIsConstrained(false);

    // Set input data
    data->setData("input_data", input_program);
    data->setData("output_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FormatAsInputTask task("abc", "input_data", "output_data", "output_data2", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_EQ(context->data_storage->getData("output_data2"), context->data_storage->getData("input_data"));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method with input is state waypoints
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    data->setData("output_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FormatAsInputTask task("abc", "input_data", "output_data", "output_data2", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_EQ(context->data_storage->getData("output_data2"), context->data_storage->getData("input_data"));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("output_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FormatAsInputTask task("abc", "input_data", "output_data", "output_data2", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data [1]
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FormatAsInputTask task("abc", "input_data", "output_data", "output_data2", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure inputs are not the same size
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("output_data", test_suite::freespaceExampleProgramABB());
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FormatAsInputTask task("abc", "input_data", "output_data", "output_data2", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerFormatAsResultTaskTests)  // NOLINT
{
  {  // Construction
    FormatAsResultTask task;
    EXPECT_EQ(task.getName(), "FormatAsResultTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             programs: [input_data]
                           outputs:
                             programs: [output_data])";
    YAML::Node config = YAML::Load(str);
    FormatAsResultTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 1);
    EXPECT_EQ(task.getInputKeys().get<std::vector<std::string>>(FormatAsResultTask::INOUT_PROGRAMS_PORT),
              std::vector<std::string>{ "input_data" });
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get<std::vector<std::string>>(FormatAsResultTask::INOUT_PROGRAMS_PORT),
              std::vector<std::string>{ "output_data" });
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsResultTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             programs: [input_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsResultTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             programs: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsResultTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             programs: input_data
                           outputs:
                             programs: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsResultTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             programs: [input_data]
                             extra: extra
                           outputs:
                             programs: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsResultTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             programs: [input_data]
                             extra: extra
                           outputs:
                             programs: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsResultTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             programs: [input_data]
                           outputs:
                             programs: [output_data]
                             extra: extra)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsResultTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<FormatAsResultTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerFormatAsResultTaskTests");
  }

  {  // Test run method
    auto data = std::make_unique<TaskComposerDataStorage>();
    CompositeInstruction compare = test_suite::jointInterpolateExampleProgramABB(false);
    CompositeInstruction input_data = test_suite::jointInterpolateExampleProgramABB(true);
    input_data.setUUID(compare.getUUID());
    for (std::size_t i = 0; i < compare.size(); ++i)
      input_data.at(i).as<MoveInstructionPoly>().setUUID(compare.at(i).as<MoveInstructionPoly>().getUUID());

    data->setData("input_data", input_data);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    std::vector<std::string> input_keys{ "input_data" };
    std::vector<std::string> output_keys{ "output_data" };
    FormatAsResultTask task("abc", input_keys, output_keys, true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_EQ(context->data_storage->getData("output_data"), compare);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data [0]
    auto data = std::make_unique<TaskComposerDataStorage>();
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    std::vector<std::string> input_keys{ "input_data" };
    std::vector<std::string> output_keys{ "output_data" };
    FormatAsResultTask task("abc", input_keys, output_keys, true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);  // Indicates an exception was thrown
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerMinLengthTaskTests)  // NOLINT
{
  {  // Construction
    MinLengthTask task;
    EXPECT_EQ(task.getName(), "MinLengthTask");
    EXPECT_EQ(task.isConditional(), false);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    MinLengthTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 3);
    EXPECT_EQ(task.getInputKeys().get(MinLengthTask::INOUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(MinLengthTask::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getInputKeys().get(MinLengthTask::INPUT_PROFILES_PORT), "profiles");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(MinLengthTask::INOUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<MinLengthTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<MinLengthTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<MinLengthTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<MinLengthTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerMinLengthTaskTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    MinLengthTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_GE(context->data_storage->getData("output_data").as<CompositeInstruction>().size(), 10);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    MinLengthTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing environment data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    MinLengthTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing profiles data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    MinLengthTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerFormatPlanningInputTaskTests)  // NOLINT
{
  {  // Construction
    FormatPlanningInputTask task;
    EXPECT_EQ(task.getName(), "FormatPlanningInputTask");
    EXPECT_EQ(task.isConditional(), false);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    FormatPlanningInputTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 2);
    EXPECT_EQ(task.getInputKeys().get(FormatPlanningInputTask::INOUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(FormatPlanningInputTask::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(MinLengthTask::INOUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatPlanningInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatPlanningInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatPlanningInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<FormatPlanningInputTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerFormatPlanningInputTaskTests");
  }

  {  // Test run method
    auto data = std::make_unique<TaskComposerDataStorage>();
    auto input_data = test_suite::jointInterpolateExampleProgramABB(true);
    data->setData("input_data", input_data);
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FormatPlanningInputTask task("abc", "input_data", "environment", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_GE(context->data_storage->getData("output_data").as<CompositeInstruction>().size(), input_data.size());
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FormatPlanningInputTask task("abc", "input_data", "environment", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing environment data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FormatPlanningInputTask task("abc", "input_data", "environment", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerFixStateBoundsTaskTests)  // NOLINT
{
  {  // Construction
    FixStateBoundsTask task;
    EXPECT_EQ(task.getName(), "FixStateBoundsTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    FixStateBoundsTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 3);
    EXPECT_EQ(task.getInputKeys().get(FixStateBoundsTask::INOUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(FixStateBoundsTask::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getInputKeys().get(FixStateBoundsTask::INPUT_PROFILES_PORT), "profiles");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(FixStateBoundsTask::INOUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FixStateBoundsTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FixStateBoundsTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FixStateBoundsTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<FixStateBoundsTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerFixStateBoundsTaskTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FixStateBoundsTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->data_storage->hasKey("output_data"));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method empty composite
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    auto program = test_suite::jointInterpolateExampleProgramABB(true);
    program.clear();
    data->setData("input_data", program);
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FixStateBoundsTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->data_storage->hasKey("output_data"));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FixStateBoundsTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing environment data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FixStateBoundsTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing profiles data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FixStateBoundsTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerFixStateCollisionTaskTests)  // NOLINT
{
  {  // Construction
    FixStateCollisionTask task;
    EXPECT_EQ(task.getName(), "FixStateCollisionTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    FixStateCollisionTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 3);
    EXPECT_EQ(task.getInputKeys().get(FixStateCollisionTask::INOUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(FixStateCollisionTask::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getInputKeys().get(FixStateCollisionTask::INPUT_PROFILES_PORT), "profiles");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(FixStateCollisionTask::INOUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FixStateCollisionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FixStateCollisionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FixStateCollisionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<FixStateCollisionTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerFixStateCollisionTaskTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FixStateCollisionTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->data_storage->hasKey("output_data"));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());

    test_suite::runSerializationTest(*node_info, "TaskComposerFixStateCollisionNodeInfoTests");
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FixStateCollisionTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing environment data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FixStateCollisionTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing profiles data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    FixStateCollisionTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerProfileSwitchTaskTests)  // NOLINT
{
  {  // Construction
    ProfileSwitchTask task;
    EXPECT_EQ(task.getName(), "ProfileSwitchTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             profiles: profiles)";
    YAML::Node config = YAML::Load(str);
    ProfileSwitchTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 2);
    EXPECT_EQ(task.getInputKeys().get(ProfileSwitchTask::INPUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(ProfileSwitchTask::INPUT_PROFILES_PORT), "profiles");
    EXPECT_EQ(task.getOutputKeys().size(), 0);
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<ProfileSwitchTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<ProfileSwitchTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerProfileSwitchTaskTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    ProfileSwitchTask task("abc", "input_data", "profiles", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    ProfileSwitchTask task("abc", "input_data", "profiles", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing profiles data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    ProfileSwitchTask task("abc", "input_data", "profiles", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerUpdateEndStateTaskTests)  // NOLINT
{
  {  // Construction
    UpdateEndStateTask task("abc", "input_data", "next_data", "output_data", false);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), false);
    EXPECT_EQ(task.getInputKeys().size(), 2);
    EXPECT_EQ(task.getInputKeys().get(UpdateEndStateTask::INPUT_CURRENT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(UpdateEndStateTask::INPUT_NEXT_PROGRAM_PORT), "next_data");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(UpdateEndStateTask::OUTPUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction
    UpdateEndStateTask task("abc", "next_data", "output_data", true);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 2);
    EXPECT_EQ(task.getInputKeys().get(UpdateEndStateTask::INPUT_CURRENT_PROGRAM_PORT), task.getUUIDString());
    EXPECT_EQ(task.getInputKeys().get(UpdateEndStateTask::INPUT_NEXT_PROGRAM_PORT), "next_data");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(UpdateEndStateTask::OUTPUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Serialization
    auto task = std::make_unique<UpdateEndStateTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerUpdateEndStateTaskTests");
  }

  {  // Test run method
    Eigen::VectorXd end_position;
    end_position.resize(6);
    end_position << 1, 2, 3, 4, 5, 6;

    auto input_program = test_suite::jointInterpolateExampleProgramABB(true);
    auto next_program = test_suite::jointInterpolateExampleProgramABB(false);
    next_program.front().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().setPosition(end_position);
    EXPECT_NE(getJointPosition(input_program.back().as<MoveInstructionPoly>().getWaypoint()),
              getJointPosition(next_program.front().as<MoveInstructionPoly>().getWaypoint()));

    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", input_program);
    data->setData("next_data", next_program);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpdateEndStateTask task("abc", "input_data", "next_data", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    auto output_program = context->data_storage->getData("output_data").as<CompositeInstruction>();
    EXPECT_EQ(output_program.back().getUUID(), input_program.back().getUUID());
    EXPECT_EQ(output_program.back().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint(), true);
    EXPECT_EQ(getJointPosition(output_program.back().as<MoveInstructionPoly>().getWaypoint()),
              getJointPosition(next_program.front().as<MoveInstructionPoly>().getWaypoint()));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("next_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpdateEndStateTask task("abc", "input_data", "next_data", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing next data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpdateEndStateTask task("abc", "input_data", "next_data", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerUpdateStartStateTaskTests)  // NOLINT
{
  {  // Construction
    UpdateStartStateTask task("abc", "input_data", "prev_data", "output_data", false);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), false);
    EXPECT_EQ(task.getInputKeys().size(), 2);
    EXPECT_EQ(task.getInputKeys().get(UpdateStartStateTask::INPUT_CURRENT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(UpdateStartStateTask::INPUT_PREVIOUS_PROGRAM_PORT), "prev_data");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(UpdateStartStateTask::OUTPUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction
    UpdateStartStateTask task("abc", "prev_data", "output_data", true);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 2);
    EXPECT_EQ(task.getInputKeys().get(UpdateStartStateTask::INPUT_CURRENT_PROGRAM_PORT), task.getUUIDString());
    EXPECT_EQ(task.getInputKeys().get(UpdateStartStateTask::INPUT_PREVIOUS_PROGRAM_PORT), "prev_data");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(UpdateStartStateTask::OUTPUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Serialization
    auto task = std::make_unique<UpdateStartStateTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerUpdateStartStateTaskTests");
  }

  {  // Test run method
    Eigen::VectorXd start_position;
    start_position.resize(6);
    start_position << 1, 2, 3, 4, 5, 6;

    auto input_program = test_suite::jointInterpolateExampleProgramABB(true);
    auto prev_program = test_suite::jointInterpolateExampleProgramABB(false);
    prev_program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().setPosition(start_position);
    EXPECT_NE(getJointPosition(input_program.front().as<MoveInstructionPoly>().getWaypoint()),
              getJointPosition(prev_program.back().as<MoveInstructionPoly>().getWaypoint()));

    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", input_program);
    data->setData("prev_data", prev_program);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpdateStartStateTask task("abc", "input_data", "prev_data", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    auto output_program = context->data_storage->getData("output_data").as<CompositeInstruction>();
    EXPECT_EQ(output_program.front().getUUID(), input_program.front().getUUID());
    EXPECT_EQ(output_program.front().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint(), true);
    EXPECT_EQ(getJointPosition(output_program.front().as<MoveInstructionPoly>().getWaypoint()),
              getJointPosition(prev_program.back().as<MoveInstructionPoly>().getWaypoint()));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("prev_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpdateStartStateTask task("abc", "input_data", "prev_data", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing prev data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpdateStartStateTask task("abc", "input_data", "prev_data", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerUpdateStartAndEndStateTaskTests)  // NOLINT
{
  {  // Construction
    UpdateStartAndEndStateTask task("abc", "input_data", "prev_data", "next_data", "output_data", false);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), false);
    EXPECT_EQ(task.getInputKeys().size(), 3);
    EXPECT_EQ(task.getInputKeys().get(UpdateStartAndEndStateTask::INPUT_CURRENT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(UpdateStartAndEndStateTask::INPUT_PREVIOUS_PROGRAM_PORT), "prev_data");
    EXPECT_EQ(task.getInputKeys().get(UpdateStartAndEndStateTask::INPUT_NEXT_PROGRAM_PORT), "next_data");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(UpdateStartAndEndStateTask::OUTPUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction
    UpdateStartAndEndStateTask task("abc", "prev_data", "next_data", "output_data", true);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 3);
    EXPECT_EQ(task.getInputKeys().get(UpdateStartAndEndStateTask::INPUT_CURRENT_PROGRAM_PORT), task.getUUIDString());
    EXPECT_EQ(task.getInputKeys().get(UpdateStartAndEndStateTask::INPUT_PREVIOUS_PROGRAM_PORT), "prev_data");
    EXPECT_EQ(task.getInputKeys().get(UpdateStartAndEndStateTask::INPUT_NEXT_PROGRAM_PORT), "next_data");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(UpdateStartAndEndStateTask::OUTPUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Serialization
    auto task = std::make_unique<UpdateStartAndEndStateTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerUpdateStartAndEndStateTaskTests");
  }

  {  // Test run method
    Eigen::VectorXd start_position, end_position;
    start_position.resize(6);
    end_position.resize(6);
    start_position << 1, 2, 3, 4, 5, 6;
    end_position << 6, 5, 4, 3, 2, 1;

    auto input_program = test_suite::jointInterpolateExampleProgramABB(true);
    auto prev_program = test_suite::jointInterpolateExampleProgramABB(false);
    auto next_program = test_suite::jointInterpolateExampleProgramABB(false);
    prev_program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().setPosition(start_position);
    EXPECT_NE(getJointPosition(input_program.front().as<MoveInstructionPoly>().getWaypoint()),
              getJointPosition(prev_program.back().as<MoveInstructionPoly>().getWaypoint()));
    next_program.front().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().setPosition(end_position);
    EXPECT_NE(getJointPosition(input_program.back().as<MoveInstructionPoly>().getWaypoint()),
              getJointPosition(next_program.front().as<MoveInstructionPoly>().getWaypoint()));

    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", input_program);
    data->setData("prev_data", prev_program);
    data->setData("next_data", next_program);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpdateStartAndEndStateTask task("abc", "input_data", "prev_data", "next_data", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    auto output_program = context->data_storage->getData("output_data").as<CompositeInstruction>();
    EXPECT_EQ(output_program.front().getUUID(), input_program.front().getUUID());
    EXPECT_EQ(output_program.front().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint(), true);
    EXPECT_EQ(getJointPosition(output_program.front().as<MoveInstructionPoly>().getWaypoint()),
              getJointPosition(prev_program.back().as<MoveInstructionPoly>().getWaypoint()));
    EXPECT_EQ(output_program.back().getUUID(), input_program.back().getUUID());
    EXPECT_EQ(output_program.back().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint(), true);
    EXPECT_EQ(getJointPosition(output_program.back().as<MoveInstructionPoly>().getWaypoint()),
              getJointPosition(next_program.front().as<MoveInstructionPoly>().getWaypoint()));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("next_data", test_suite::jointInterpolateExampleProgramABB(false));
    data->setData("prev_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpdateStartAndEndStateTask task("abc", "input_data", "prev_data", "next_data", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing prev data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("next_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpdateStartAndEndStateTask task("abc", "input_data", "prev_data", "next_data", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing next data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("prev_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpdateStartAndEndStateTask task("abc", "input_data", "prev_data", "next_data", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerUpsampleTrajectoryTaskTests)  // NOLINT
{
  {  // Construction
    UpsampleTrajectoryTask task;
    EXPECT_EQ(task.getName(), "UpsampleTrajectoryTask");
    EXPECT_EQ(task.isConditional(), false);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             profiles: profiles
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    UpsampleTrajectoryTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 2);
    EXPECT_EQ(task.getInputKeys().get(UpsampleTrajectoryTask::INOUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(UpsampleTrajectoryTask::INPUT_PROFILES_PORT), "profiles");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(UpsampleTrajectoryTask::INOUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<UpsampleTrajectoryTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             profiles: profiles)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<UpsampleTrajectoryTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<UpsampleTrajectoryTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<UpsampleTrajectoryTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerUpsampleTrajectoryTaskTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpsampleTrajectoryTask task("abc", "input_data", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_EQ(context->data_storage->getData("output_data").as<CompositeInstruction>().size(), 18);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpsampleTrajectoryTask task("abc", "input_data", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing profiles data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    UpsampleTrajectoryTask task("abc", "input_data", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerIterativeSplineParameterizationTaskTests)  // NOLINT
{
  {  // Construction
    IterativeSplineParameterizationTask task;
    EXPECT_EQ(task.getName(), "IterativeSplineParameterizationTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data
                           add_points: true)";
    YAML::Node config = YAML::Load(str);
    IterativeSplineParameterizationTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 3);
    EXPECT_EQ(task.getInputKeys().get(IterativeSplineParameterizationTask::INOUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(IterativeSplineParameterizationTask::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getInputKeys().get(IterativeSplineParameterizationTask::INPUT_PROFILES_PORT), "profiles");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(IterativeSplineParameterizationTask::INOUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_unique<IterativeSplineParameterizationTask>("abc", config["config"], factory));
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles)";
    YAML::Node config = YAML::Load(str);
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_unique<IterativeSplineParameterizationTask>("abc", config["config"], factory));
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_unique<IterativeSplineParameterizationTask>("abc", config["config"], factory));
  }

  {  // Serialization
    auto task = std::make_unique<IterativeSplineParameterizationTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerIterativeSplineParameterizationTaskTests");
  }

  {  // Test run method
    // Create input data
    auto data = std::make_unique<TaskComposerDataStorage>();
    {
      auto profiles = std::make_shared<ProfileDictionary>();
      auto data2 = std::make_unique<TaskComposerDataStorage>();
      data2->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
      data2->setData("profiles", profiles);
      auto context = std::make_unique<TaskComposerContext>("abc", std::move(data2));
      UpsampleTrajectoryTask task("abc", "input_data", "profiles", "output_data", true);
      EXPECT_EQ(task.run(*context), 1);
      data->setData("input_data", context->data_storage->getData("output_data"));
      EXPECT_EQ(context->data_storage->getData("output_data").as<CompositeInstruction>().size(), 18);
    }
    auto profiles = std::make_shared<ProfileDictionary>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    IterativeSplineParameterizationTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_EQ(context->data_storage->getData("output_data").as<CompositeInstruction>().size(), 18);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    auto program = test_suite::jointInterpolateExampleProgramABB(false);
    program.clear();
    data->setData("input_data", program);
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    IterativeSplineParameterizationTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->data_storage->hasKey("output_data"));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    IterativeSplineParameterizationTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing enviroment data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    IterativeSplineParameterizationTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing profiles data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    IterativeSplineParameterizationTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerTimeOptimalParameterizationTaskTests)  // NOLINT
{
  {  // Construction
    TimeOptimalParameterizationTask task;
    EXPECT_EQ(task.getName(), "TimeOptimalParameterizationTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    TimeOptimalParameterizationTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 3);
    EXPECT_EQ(task.getInputKeys().get(TimeOptimalParameterizationTask::INOUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(TimeOptimalParameterizationTask::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getInputKeys().get(TimeOptimalParameterizationTask::INPUT_PROFILES_PORT), "profiles");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(TimeOptimalParameterizationTask::INOUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TimeOptimalParameterizationTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                             manip_info: manip_info
                             composite_profile_remapping: composite_profile_remapping
                             move_profile_remapping: move_profile_remapping)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TimeOptimalParameterizationTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TimeOptimalParameterizationTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<TimeOptimalParameterizationTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerTimeOptimalParameterizationTaskTests");
  }

  {  // Test run method
    // Create input data
    auto data = std::make_unique<TaskComposerDataStorage>();
    {
      auto profiles = std::make_shared<ProfileDictionary>();
      auto data2 = std::make_unique<TaskComposerDataStorage>();
      data2->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
      data2->setData("profiles", profiles);
      auto context = std::make_unique<TaskComposerContext>("abc", std::move(data2));
      UpsampleTrajectoryTask task("abc", "input_data", "profiles", "output_data", true);
      EXPECT_EQ(task.run(*context), 1);
      data->setData("input_data", context->data_storage->getData("output_data"));
      EXPECT_EQ(context->data_storage->getData("output_data").as<CompositeInstruction>().size(), 18);
    }
    auto profiles = std::make_shared<ProfileDictionary>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    TimeOptimalParameterizationTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_EQ(context->data_storage->getData("output_data").as<CompositeInstruction>().size(), 18);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());

    test_suite::runSerializationTest(*node_info, "TaskComposerTimeOptimalParameterizationNodeInfoTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    auto program = test_suite::jointInterpolateExampleProgramABB(false);
    program.clear();
    data->setData("input_data", program);
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    TimeOptimalParameterizationTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->data_storage->hasKey("output_data"));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    TimeOptimalParameterizationTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing environment data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    TimeOptimalParameterizationTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing profiles data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    TimeOptimalParameterizationTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerRuckigTrajectorySmoothingTaskTests)  // NOLINT
{
  {  // Construction
    RuckigTrajectorySmoothingTask task;
    EXPECT_EQ(task.getName(), "RuckigTrajectorySmoothingTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    RuckigTrajectorySmoothingTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 3);
    EXPECT_EQ(task.getInputKeys().get(RuckigTrajectorySmoothingTask::INOUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(RuckigTrajectorySmoothingTask::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getInputKeys().get(RuckigTrajectorySmoothingTask::INPUT_PROFILES_PORT), "profiles");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(RuckigTrajectorySmoothingTask::INOUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RuckigTrajectorySmoothingTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                             manip_info: manip_info
                             composite_profile_remapping: composite_profile_remapping
                             move_profile_remapping: move_profile_remapping)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RuckigTrajectorySmoothingTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RuckigTrajectorySmoothingTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<RuckigTrajectorySmoothingTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerRuckigTrajectorySmoothingTaskTests");
  }

  {  // Test run method
    // Create input data
    auto data = std::make_unique<TaskComposerDataStorage>();
    {
      auto profiles = std::make_shared<ProfileDictionary>();
      auto data2 = std::make_unique<TaskComposerDataStorage>();
      data2->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
      data2->setData("profiles", profiles);
      auto context = std::make_unique<TaskComposerContext>("abc", std::move(data2));
      UpsampleTrajectoryTask task("abc", "input_data", "profiles", "output_data", true);
      EXPECT_EQ(task.run(*context), 1);
      EXPECT_EQ(context->data_storage->getData("output_data").as<CompositeInstruction>().size(), 18);

      auto data3 = std::make_unique<TaskComposerDataStorage>();
      data3->setData("input_data", context->data_storage->getData("output_data"));
      data3->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
      data3->setData("profiles", profiles);
      auto context2 = std::make_unique<TaskComposerContext>("abc", std::move(data3));
      TimeOptimalParameterizationTask task2("abc", "input_data", "environment", "profiles", "output_data", true);
      EXPECT_EQ(task2.run(*context2), 1);
      data->setData("input_data", context2->data_storage->getData("output_data"));
      EXPECT_EQ(context2->data_storage->getData("output_data").as<CompositeInstruction>().size(), 18);
    }
    auto profiles = std::make_shared<ProfileDictionary>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    RuckigTrajectorySmoothingTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_EQ(context->data_storage->getData("output_data").as<CompositeInstruction>().size(), 18);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    auto program = test_suite::jointInterpolateExampleProgramABB(false);
    program.clear();
    data->setData("input_data", program);
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    RuckigTrajectorySmoothingTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->data_storage->hasKey("output_data"));
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    RuckigTrajectorySmoothingTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing environment data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    RuckigTrajectorySmoothingTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing profiles data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    RuckigTrajectorySmoothingTask task("abc", "input_data", "environment", "profiles", "output_data", true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerMotionPlannerTaskTests)  // NOLINT
{
  {  // Construction
    MotionPlannerTask<TrajOptMotionPlanner> task;
    EXPECT_FALSE(task.getName().empty());
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data
                           format_result_as_input: false)";
    YAML::Node config = YAML::Load(str);
    MotionPlannerTask<TrajOptMotionPlanner> task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 3);
    EXPECT_EQ(task.getInputKeys().get(MotionPlannerTask<TrajOptMotionPlanner>::INOUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(MotionPlannerTask<TrajOptMotionPlanner>::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getInputKeys().get(MotionPlannerTask<TrajOptMotionPlanner>::INPUT_PROFILES_PORT), "profiles");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(MotionPlannerTask<TrajOptMotionPlanner>::INOUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_unique<MotionPlannerTask<TrajOptMotionPlanner>>("abc", config["config"], factory));
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles)";
    YAML::Node config = YAML::Load(str);
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_unique<MotionPlannerTask<TrajOptMotionPlanner>>("abc", config["config"], factory));
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_unique<MotionPlannerTask<TrajOptMotionPlanner>>("abc", config["config"], factory));
  }

  {  // Serialization
    auto task = std::make_unique<MotionPlannerTask<TrajOptMotionPlanner>>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerMotionPlannerTaskTests");
  }

  {  // Test run method
    auto data = std::make_unique<TaskComposerDataStorage>();
    {
      auto profiles = std::make_shared<ProfileDictionary>();
      auto data2 = std::make_unique<TaskComposerDataStorage>();
      data2->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
      data2->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
      data2->setData("profiles", profiles);
      auto context = std::make_unique<TaskComposerContext>("abc", std::move(data2));
      MinLengthTask task("abc", "input_data", "environment", "profiles", "output_data", true);
      EXPECT_EQ(task.run(*context), 1);
      data->setData("input_data", context->data_storage->getData("output_data"));
      EXPECT_GE(context->data_storage->getData("output_data").as<CompositeInstruction>().size(), 10);
    }
    tesseract_planning::TaskComposerLog log("TaskComposerMotionPlannerNodeInfoTests");
    auto profiles = std::make_shared<ProfileDictionary>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    log.initial_data = *data;

    log.context = std::make_shared<TaskComposerContext>("abc", std::move(data));
    MotionPlannerTask<TrajOptMotionPlanner> task(
        "abc", "input_data", "environment", "profiles", "output_data", false, true);
    EXPECT_EQ(task.run(*log.context), 1);
    log.dotgraph = task.getDotgraph(log.context->task_infos.getInfoMap());
    auto node_info = log.context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(log.context->isAborted(), false);
    EXPECT_EQ(log.context->isSuccessful(), true);
    EXPECT_GE(log.context->data_storage->getData("output_data").as<CompositeInstruction>().size(), 10);
    EXPECT_TRUE(log.context->task_infos.getAbortingNode().is_nil());

    test_suite::runSerializationTest(*node_info, "TaskComposerMotionPlannerNodeInfoTests");
    {
      const std::string filepath = tesseract_common::getTempPath() + "TaskComposerMotionPlannerLogTests.xml";
      tesseract_common::Serialization::toArchiveFileXML<tesseract_planning::TaskComposerLog>(log, filepath);
      auto ninput = tesseract_common::Serialization::fromArchiveFileXML<tesseract_planning::TaskComposerLog>(filepath);
      EXPECT_TRUE(ninput.initial_data.hasKey("environment"));
    }
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    MotionPlannerTask<TrajOptMotionPlanner> task(
        "abc", "input_data", "environment", "profiles", "output_data", false, true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing environment data
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    data->setData("profiles", profiles);
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    MotionPlannerTask<TrajOptMotionPlanner> task(
        "abc", "input_data", "environment", "profiles", "output_data", false, true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing profiles data
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    MotionPlannerTask<TrajOptMotionPlanner> task(
        "abc", "input_data", "environment", "profiles", "output_data", false, true);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerRasterMotionTaskTests)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  std::filesystem::path config_path(
      locator_->locateResource("package://tesseract_task_composer/config/task_composer_plugins.yaml")->getFilePath());
  TaskComposerPluginFactory factory(config_path, locator);

  {  // Construction
    RasterMotionTask task;
    EXPECT_EQ(task.getName(), "RasterMotionTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    RasterMotionTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 2);
    EXPECT_EQ(task.getInputKeys().get(RasterMotionTask::INOUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(RasterMotionTask::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(RasterMotionTask::INOUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             task: CartesianPipeline)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<RasterMotionTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerRasterMotionTaskTests");
  }

  {  // Test run method
    // Create raster task
    TaskComposerNode::UPtr task = factory.createTaskComposerNode("RasterFtPipeline");
    const std::string output_key = { "output_data" };

    // Define profiles
    auto profiles = std::make_shared<ProfileDictionary>();

    // Create Data Storage
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData("planning_input", test_suite::rasterExampleProgram());
    data_storage->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data_storage->setData("profiles", profiles);

    // Solve raster plan
    auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");
    TaskComposerFuture::UPtr future = executor->run(*task, std::move(data_storage), true);
    future->wait();

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "TaskComposerRasterMotionTaskTests.dot");
    EXPECT_NO_THROW(task->dump(os1, nullptr, future->context->task_infos.getInfoMap()));  // NOLINT
    os1.close();

    EXPECT_EQ(future->context->isAborted(), false);
    EXPECT_EQ(future->context->isSuccessful(), true);
    EXPECT_TRUE(future->context->data_storage->hasKey(output_key));
    EXPECT_TRUE(future->context->task_infos.getAbortingNode().is_nil());
    auto info_map = future->context->task_infos.getInfoMap();
    EXPECT_EQ(info_map.size(), 77);

    // Make sure keys are unique
    std::vector<std::string> raster_input_keys;
    std::vector<std::string> raster_output_keys;
    std::vector<std::string> update_input_keys;
    std::vector<std::string> update_output_keys;
    std::vector<std::string> transition_input_keys;
    std::vector<std::string> transition_output_keys;
    for (const auto& info : info_map)
    {
      std::cout << info.second.name << "\n";
      if (boost::algorithm::starts_with(info.second.name, "Raster #"))
      {
        auto it1 = std::find(raster_input_keys.begin(), raster_input_keys.end(), info.second.input_keys.get("program"));
        ASSERT_TRUE(it1 == raster_input_keys.end());
        raster_input_keys.push_back(info.second.input_keys.get("program"));

        auto it2 =
            std::find(raster_output_keys.begin(), raster_output_keys.end(), info.second.output_keys.get("program"));
        ASSERT_TRUE(it2 == raster_output_keys.end());
        raster_output_keys.push_back(info.second.output_keys.get("program"));
      }
      else if (info.second.name == "UpdateStartAndEndStateTask" || info.second.name == "UpdateStartStateTask" ||
               info.second.name == "UpdateEndStateTask")
      {
        auto it1 = std::find(
            update_input_keys.begin(), update_input_keys.end(), info.second.input_keys.get("current_program"));
        ASSERT_TRUE(it1 == update_input_keys.end());
        update_input_keys.push_back(info.second.input_keys.get("current_program"));

        auto it2 =
            std::find(update_output_keys.begin(), update_output_keys.end(), info.second.output_keys.get("program"));
        ASSERT_TRUE(it2 == update_output_keys.end());
        update_output_keys.push_back(info.second.output_keys.get("program"));
      }
      else if (boost::algorithm::starts_with(info.second.name, "Transition #") ||
               boost::algorithm::starts_with(info.second.name, "From Start") ||
               boost::algorithm::starts_with(info.second.name, "To End"))
      {
        auto it1 = std::find(
            transition_input_keys.begin(), transition_input_keys.end(), info.second.input_keys.get("program"));
        ASSERT_TRUE(it1 == transition_input_keys.end());
        transition_input_keys.push_back(info.second.input_keys.get("program"));

        auto it2 = std::find(
            transition_output_keys.begin(), transition_output_keys.end(), info.second.output_keys.get("program"));
        ASSERT_TRUE(it2 == transition_output_keys.end());
        transition_output_keys.push_back(info.second.output_keys.get("program"));
      }
    }
    EXPECT_FALSE(raster_input_keys.empty());
    EXPECT_FALSE(raster_output_keys.empty());
    EXPECT_FALSE(update_input_keys.empty());
    EXPECT_FALSE(update_output_keys.empty());
    EXPECT_FALSE(transition_input_keys.empty());
    EXPECT_FALSE(transition_output_keys.empty());
  }

  {  // Failure missing input data
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    RasterMotionTask task("abc", config["config"], factory);

    // Create Data Storage
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);

    // Solve
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure null input data
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    RasterMotionTask task("abc", config["config"], factory);

    // Create data storage
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", tesseract_common::AnyPoly());
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);

    // Solve
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure input data is not composite
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    RasterMotionTask task("abc", config["config"], factory);

    // Create data storage
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", tesseract_common::AnyPoly(tesseract_common::JointState()));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);

    // Solve
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure input data is not composite
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           freespace:
                             task: FreespacePipeline
                             config:
                               indexing: [input_data, output_data]
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    RasterMotionTask task("abc", config["config"], factory);

    // Create data storage
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);

    // Solve
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerRasterOnlyMotionTaskTests)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  std::filesystem::path config_path(
      locator_->locateResource("package://tesseract_task_composer/config/task_composer_plugins.yaml")->getFilePath());
  TaskComposerPluginFactory factory(config_path, locator);

  {  // Construction
    RasterOnlyMotionTask task;
    EXPECT_EQ(task.getName(), "RasterOnlyMotionTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    RasterOnlyMotionTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 2);
    EXPECT_EQ(task.getInputKeys().get(RasterOnlyMotionTask::INOUT_PROGRAM_PORT), "input_data");
    EXPECT_EQ(task.getInputKeys().get(RasterOnlyMotionTask::INPUT_ENVIRONMENT_PORT), "environment");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().get(RasterOnlyMotionTask::INOUT_PROGRAM_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterOnlyMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterOnlyMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             program: output_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterOnlyMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterOnlyMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             task: CartesianPipeline)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterOnlyMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterOnlyMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterOnlyMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterOnlyMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterOnlyMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RasterOnlyMotionTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<RasterOnlyMotionTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerRasterOnlyMotionTaskTests");
  }

  {  // Test run method
    // Create raster task
    TaskComposerNode::UPtr task = factory.createTaskComposerNode("RasterFtOnlyPipeline");
    const std::string output_key{ "output_data" };

    // Define profiles
    auto profiles = std::make_shared<ProfileDictionary>();

    // Create Data Storage
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData("planning_input", test_suite::rasterOnlyExampleProgram());
    data_storage->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data_storage->setData("profiles", profiles);

    // Solve raster plan
    auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");
    TaskComposerFuture::UPtr future = executor->run(*task, std::move(data_storage), true);
    future->wait();

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "TaskComposerRasterOnlyMotionTaskTests.dot");
    EXPECT_NO_THROW(task->dump(os1, nullptr, future->context->task_infos.getInfoMap()));  // NOLINT
    os1.close();

    EXPECT_EQ(future->context->isAborted(), false);
    EXPECT_EQ(future->context->isSuccessful(), true);
    EXPECT_TRUE(future->context->data_storage->hasKey(output_key));
    EXPECT_TRUE(future->context->task_infos.getAbortingNode().is_nil());
    auto info_map = future->context->task_infos.getInfoMap();
    EXPECT_EQ(info_map.size(), 61);

    // Make sure keys are unique
    std::vector<std::string> raster_input_keys;
    std::vector<std::string> raster_output_keys;
    std::vector<std::string> update_input_keys;
    std::vector<std::string> update_output_keys;
    std::vector<std::string> transition_input_keys;
    std::vector<std::string> transition_output_keys;
    for (const auto& info : info_map)
    {
      if (boost::algorithm::starts_with(info.second.name, "Raster #"))
      {
        auto it1 = std::find(raster_input_keys.begin(), raster_input_keys.end(), info.second.input_keys.get("program"));
        ASSERT_TRUE(it1 == raster_input_keys.end());
        raster_input_keys.push_back(info.second.input_keys.get("program"));

        auto it2 =
            std::find(raster_output_keys.begin(), raster_output_keys.end(), info.second.output_keys.get("program"));
        ASSERT_TRUE(it2 == raster_output_keys.end());
        raster_output_keys.push_back(info.second.output_keys.get("program"));
      }
      else if (info.second.name == "UpdateStartAndEndStateTask" || info.second.name == "UpdateStartStateTask" ||
               info.second.name == "UpdateEndStateTask")
      {
        auto it1 = std::find(
            update_input_keys.begin(), update_input_keys.end(), info.second.input_keys.get("current_program"));
        ASSERT_TRUE(it1 == update_input_keys.end());
        update_input_keys.push_back(info.second.input_keys.get("current_program"));

        auto it2 =
            std::find(update_output_keys.begin(), update_output_keys.end(), info.second.output_keys.get("program"));
        ASSERT_TRUE(it2 == update_output_keys.end());
        update_output_keys.push_back(info.second.output_keys.get("program"));
      }
      else if (boost::algorithm::starts_with(info.second.name, "Transition #") ||
               boost::algorithm::starts_with(info.second.name, "From Start") ||
               boost::algorithm::starts_with(info.second.name, "To End"))
      {
        auto it1 = std::find(
            transition_input_keys.begin(), transition_input_keys.end(), info.second.input_keys.get("program"));
        ASSERT_TRUE(it1 == transition_input_keys.end());
        transition_input_keys.push_back(info.second.input_keys.get("program"));

        auto it2 = std::find(
            transition_output_keys.begin(), transition_output_keys.end(), info.second.output_keys.get("program"));
        ASSERT_TRUE(it2 == transition_output_keys.end());
        transition_output_keys.push_back(info.second.output_keys.get("program"));
      }
    }
    EXPECT_FALSE(raster_input_keys.empty());
    EXPECT_FALSE(raster_output_keys.empty());
    EXPECT_FALSE(update_input_keys.empty());
    EXPECT_FALSE(update_output_keys.empty());
    EXPECT_FALSE(transition_input_keys.empty());
    EXPECT_FALSE(transition_output_keys.empty());
  }

  {  // Failure missing input data
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    RasterOnlyMotionTask task("abc", config["config"], factory);

    // Create Data Storage
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);

    // Solve
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure null input data
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    RasterOnlyMotionTask task("abc", config["config"], factory);

    // Create data storage
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", tesseract_common::AnyPoly());
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);

    // Solve
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure input data is not composite
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    RasterOnlyMotionTask task("abc", config["config"], factory);

    // Create data storage
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", tesseract_common::AnyPoly(tesseract_common::JointState()));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);

    // Solve
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure input data is not composite
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                           outputs:
                             program: output_data
                           raster:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data]
                           transition:
                             task: CartesianPipeline
                             config:
                               indexing: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    RasterOnlyMotionTask task("abc", config["config"], factory);

    // Create data storage
    auto profiles = std::make_shared<ProfileDictionary>();
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);

    // Solve
    auto context = std::make_unique<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
