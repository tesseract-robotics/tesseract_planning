#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_task_composer/planning/nodes/check_input_task.h>
#include <tesseract_task_composer/planning/nodes/continuous_contact_check_task.h>
#include <tesseract_task_composer/planning/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/planning/nodes/format_as_input_task.h>

#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>

#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/test_suite/task_composer_serialization_utils.hpp>
#include <tesseract_task_composer/core/test_suite/test_programs.hpp>

#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

#include <tesseract_common/types.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/manipulator_info.h>

#include <tesseract_environment/environment.h>

using namespace tesseract_planning;

class TesseractTaskComposerPlanningUnit : public ::testing::Test
{
protected:
  tesseract_environment::Environment::Ptr env_;
  tesseract_common::ManipulatorInfo manip_;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    auto env = std::make_shared<tesseract_environment::Environment>();

    tesseract_common::fs::path urdf_path(
        locator->locateResource("package://tesseract_support/urdf/abb_irb2400.urdf")->getFilePath());
    tesseract_common::fs::path srdf_path(
        locator->locateResource("package://tesseract_support/urdf/abb_irb2400.srdf")->getFilePath());
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;

    manip_.manipulator = "manipulator";
  }
};

TEST_F(TesseractTaskComposerPlanningUnit, TaskComposerCheckInputTaskTests)  // NOLINT
{
  {  // Construction
    CheckInputTask task;
    EXPECT_EQ(task.getName(), "CheckInputTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    CheckInputTask task("TaskComposerCheckInputTaskTests", "input_data", true);
    EXPECT_EQ(task.getName(), "TaskComposerCheckInputTaskTests");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 1);
    EXPECT_EQ(task.getInputKeys().front(), "input_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: [input_data])";
    YAML::Node config = YAML::Load(str);
    CheckInputTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 1);
    EXPECT_EQ(task.getInputKeys().front(), "input_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<CheckInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<CheckInputTask>("abc", "input_data", true);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerCheckInputTaskTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    data.setData("input_data", test_suite::freespaceExampleProgramABB());
    auto problem =
        std::make_unique<PlanningTaskComposerProblem>(env_, manip_, data, profiles, "TaskComposerCheckInputTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    CheckInputTask task("TaskComposerCheckInputTaskTests", "input_data", true);
    EXPECT_EQ(task.run(*input), 1);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->message, "Successful");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    auto problem =
        std::make_unique<PlanningTaskComposerProblem>(env_, manip_, data, profiles, "TaskComposerCheckInputTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    CheckInputTask task("TaskComposerCheckInputTaskTests", "input_data", true);
    EXPECT_EQ(task.run(*input), 0);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  }
}

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
                           inputs: [input_data])";
    YAML::Node config = YAML::Load(str);
    ContinuousContactCheckTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 1);
    EXPECT_EQ(task.getInputKeys().front(), "input_data");
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
    auto task = std::make_unique<ContinuousContactCheckTask>("abc", "input_data", true);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerContinuousContactCheckTaskTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    data.setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerContinuousContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    ContinuousContactCheckTask task("TaskComposerContinuousContactCheckTaskTests", "input_data", true);
    EXPECT_EQ(task.run(*input), 1);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());

    // Serialization
    test_suite::runSerializationTest(dynamic_cast<const ContinuousContactCheckTaskInfo&>(*node_info),
                                     "TaskComposerContinuousContactCheckNodeInfoTests");
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerContinuousContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    ContinuousContactCheckTask task("TaskComposerContinuousContactCheckTaskTests", "input_data", true);
    EXPECT_EQ(task.run(*input), 0);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure collision
    auto profiles = std::make_shared<ProfileDictionary>();

    auto profile = std::make_unique<ContactCheckProfile>();
    profile->config.contact_manager_config = tesseract_collision::ContactManagerConfig(1.5);
    profile->config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    profiles->addProfile<ContactCheckProfile>(
        "TaskComposerContinuousContactCheckTaskTests", DEFAULT_PROFILE_KEY, std::move(profile));

    TaskComposerDataStorage data;
    data.setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerContinuousContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    ContinuousContactCheckTask task("TaskComposerContinuousContactCheckTaskTests", "input_data", true);
    EXPECT_EQ(task.run(*input), 0);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(dynamic_cast<const ContinuousContactCheckTaskInfo&>(*node_info).contact_results.empty(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
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
                           inputs: [input_data])";
    YAML::Node config = YAML::Load(str);
    DiscreteContactCheckTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 1);
    EXPECT_EQ(task.getInputKeys().front(), "input_data");
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
    auto task = std::make_unique<DiscreteContactCheckTask>("abc", "input_data", true);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerDiscreteContactCheckTaskTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    data.setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerDiscreteContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    DiscreteContactCheckTask task("TaskComposerDiscreteContactCheckTaskTests", "input_data", true);
    EXPECT_EQ(task.run(*input), 1);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());

    // Serialization
    test_suite::runSerializationTest(dynamic_cast<const DiscreteContactCheckTaskInfo&>(*node_info),
                                     "TaskComposerDiscreteContactCheckNodeInfoTests");
  }

  {  // Failure missing input data
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerDiscreteContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    DiscreteContactCheckTask task("TaskComposerDiscreteContactCheckTaskTests", "input_data", true);
    EXPECT_EQ(task.run(*input), 0);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure collision
    auto profiles = std::make_shared<ProfileDictionary>();

    auto profile = std::make_unique<ContactCheckProfile>();
    profile->config.contact_manager_config = tesseract_collision::ContactManagerConfig(1.5);
    profile->config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    profiles->addProfile<ContactCheckProfile>(
        "TaskComposerDiscreteContactCheckTaskTests", DEFAULT_PROFILE_KEY, std::move(profile));

    TaskComposerDataStorage data;
    data.setData("input_data", test_suite::jointInterpolateExampleProgramABB());
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerDiscreteContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    DiscreteContactCheckTask task("TaskComposerDiscreteContactCheckTaskTests", "input_data", true);
    EXPECT_EQ(task.run(*input), 0);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(dynamic_cast<const DiscreteContactCheckTaskInfo&>(*node_info).contact_results.empty(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
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
                           inputs: [input_data, output_data]
                           outputs: [output_data])";
    YAML::Node config = YAML::Load(str);
    FormatAsInputTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputKeys().size(), 2);
    EXPECT_EQ(task.getInputKeys().front(), "input_data");
    EXPECT_EQ(task.getInputKeys().back(), "output_data");
    EXPECT_EQ(task.getOutputKeys().size(), 1);
    EXPECT_EQ(task.getOutputKeys().front(), "output_data");
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
                           inputs: [input_data, output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: [input_data, output_data]
                           outputs: [output_data, output_data2])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: [input_data, output_data, output_data2]
                           outputs: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<FormatAsInputTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<FormatAsInputTask>();

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerFormatAsInputTaskTests");
  }

  {  // Test run method
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    data.setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data.setData("output_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerDiscreteContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    std::array<std::string, 2> input_keys{ "input_data", "output_data" };
    FormatAsInputTask task("abc", input_keys, "output_data2", true);
    EXPECT_EQ(task.run(*input), 1);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_EQ(input->data_storage.getData("output_data2"), data.getData("input_data"));
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method with cartesian in input
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    auto input_program = test_suite::jointInterpolateExampleProgramABB(true);

    // Modify to have cartesian waypoint
    MoveInstructionPoly last = input_program.back().as<MoveInstructionPoly>();
    const auto& jwp = last.getWaypoint().as<JointWaypointPoly>();
    CartesianWaypoint cwp{ Eigen::Isometry3d::Identity() };
    cwp.setSeed(tesseract_common::JointState(jwp.getNames(), jwp.getPosition()));
    last.assignCartesianWaypoint(cwp);
    input_program.back() = last;
    EXPECT_EQ(input_program.size(), 2);

    // Set input data
    data.setData("input_data", input_program);
    data.setData("output_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerDiscreteContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    std::array<std::string, 2> input_keys{ "input_data", "output_data" };
    FormatAsInputTask task("abc", input_keys, "output_data2", true);
    EXPECT_EQ(task.run(*input), 1);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_EQ(input->data_storage.getData("output_data2"), data.getData("input_data"));
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method with unconstraint joint waypoint
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    auto input_program = test_suite::jointInterpolateExampleProgramABB(true);

    // Modify to have unconstrained joint waypoint
    auto& jwp = input_program.back().as<MoveInstructionPoly>().getWaypoint().as<JointWaypointPoly>();
    jwp.setIsConstrained(false);

    // Set input data
    data.setData("input_data", input_program);
    data.setData("output_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerDiscreteContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    std::array<std::string, 2> input_keys{ "input_data", "output_data" };
    FormatAsInputTask task("abc", input_keys, "output_data2", true);
    EXPECT_EQ(task.run(*input), 1);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_EQ(input->data_storage.getData("output_data2"), data.getData("input_data"));
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method with input is state waypoints
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    data.setData("input_data", test_suite::jointInterpolateExampleProgramABB(false));
    data.setData("output_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerDiscreteContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    std::array<std::string, 2> input_keys{ "input_data", "output_data" };
    FormatAsInputTask task("abc", input_keys, "output_data2", true);
    EXPECT_EQ(task.run(*input), 1);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_EQ(input->data_storage.getData("output_data2"), data.getData("input_data"));
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data [0]
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    data.setData("output_data", test_suite::jointInterpolateExampleProgramABB(false));
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerDiscreteContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    std::array<std::string, 2> input_keys{ "input_data", "output_data" };
    FormatAsInputTask task("abc", input_keys, "output_data2", true);
    EXPECT_EQ(task.run(*input), 0);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure missing input data [1]
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    data.setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerDiscreteContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    std::array<std::string, 2> input_keys{ "input_data", "output_data" };
    FormatAsInputTask task("abc", input_keys, "output_data2", true);
    EXPECT_EQ(task.run(*input), 0);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  }

  {  // Failure inputs are not the same size
    auto profiles = std::make_shared<ProfileDictionary>();
    TaskComposerDataStorage data;
    data.setData("input_data", test_suite::jointInterpolateExampleProgramABB(true));
    data.setData("output_data", test_suite::freespaceExampleProgramABB());
    auto problem = std::make_unique<PlanningTaskComposerProblem>(
        env_, manip_, data, profiles, "TaskComposerDiscreteContactCheckTaskTests");
    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
    std::array<std::string, 2> input_keys{ "input_data", "output_data" };
    FormatAsInputTask task("abc", input_keys, "output_data2", true);
    EXPECT_EQ(task.run(*input), 0);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->message.empty(), false);
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), false);
    EXPECT_EQ(input->isSuccessful(), true);
    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  }

  //  {  // Failure collision
  //    auto profiles = std::make_shared<ProfileDictionary>();

  //    auto profile = std::make_unique<ContactCheckProfile>();
  //    profile->config.contact_manager_config = tesseract_collision::ContactManagerConfig(1.5);
  //    profile->config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  //    profiles->addProfile<ContactCheckProfile>(
  //        "TaskComposerDiscreteContactCheckTaskTests", DEFAULT_PROFILE_KEY, std::move(profile));

  //    TaskComposerDataStorage data;
  //    data.setData("input_data", test_suite::jointInterpolateExampleProgramABB());
  //    auto problem = std::make_unique<PlanningTaskComposerProblem>(
  //        env_, manip_, data, profiles, "TaskComposerDiscreteContactCheckTaskTests");
  //    auto input = std::make_unique<TaskComposerInput>(std::move(problem));
  //    DiscreteContactCheckTask task("TaskComposerDiscreteContactCheckTaskTests", "input_data", true);
  //    EXPECT_EQ(task.run(*input), 0);
  //    auto node_info = input->task_infos.getInfo(task.getUUID());
  //    EXPECT_EQ(node_info->color, "red");
  //    EXPECT_EQ(node_info->return_value, 0);
  //    EXPECT_EQ(node_info->message.empty(), false);
  //    EXPECT_EQ(node_info->isAborted(), false);
  //    EXPECT_EQ(dynamic_cast<const DiscreteContactCheckTaskInfo&>(*node_info).contact_results.empty(), false);
  //    EXPECT_EQ(input->isAborted(), false);
  //    EXPECT_EQ(input->isSuccessful(), true);
  //    EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
  //  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
