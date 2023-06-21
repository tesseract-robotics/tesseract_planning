#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <sstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/joint_state.h>

#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include "serialization_utils.hpp"

TESSERACT_ANY_EXPORT(tesseract_common, JointState)

using namespace tesseract_planning;

TEST(TesseractTaskComposerCoreUnit, TaskComposerDataStorageTests)  // NOLINT
{
  std::string key{ "joint_state" };
  std::vector<std::string> joint_names{ "joint_1", "joint_2" };
  Eigen::Vector2d joint_values(5, 10);
  tesseract_common::JointState js(joint_names, joint_values);
  TaskComposerDataStorage data;
  EXPECT_FALSE(data.hasKey(key));

  // Test Add
  data.setData(key, js);
  EXPECT_TRUE(data.hasKey(key));
  EXPECT_TRUE(data.getData().size() == 1);
  EXPECT_TRUE(data.getData(key).as<tesseract_common::JointState>() == js);

  // Test Copy
  TaskComposerDataStorage copy{ data };
  EXPECT_TRUE(copy.hasKey(key));
  EXPECT_TRUE(copy.getData().size() == 1);
  EXPECT_TRUE(copy.getData(key).as<tesseract_common::JointState>() == js);

  // Serialization
  tesseract_planning::test_suite::runSerializationTest<TaskComposerDataStorage>(data, "TaskComposerDataStorageTests");

  // Test Remove
  data.removeData(key);
  EXPECT_FALSE(data.hasKey(key));
  EXPECT_TRUE(data.getData().empty());
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerInputTests)  // NOLINT
{
  TaskComposerNode node;
  auto input = std::make_unique<TaskComposerInput>(std::make_unique<TaskComposerProblem>());
  EXPECT_FALSE(input->dotgraph);
  EXPECT_FALSE(input->isAborted());
  EXPECT_TRUE(input->isSuccessful());
  EXPECT_TRUE(input->task_infos.getInfoMap().empty());
  input->task_infos.addInfo(std::make_unique<TaskComposerNodeInfo>(node, *input));
  input->abort(node.getUUID());
  EXPECT_EQ(input->task_infos.getAbortingNode(), node.getUUID());
  EXPECT_TRUE(input->isAborted());
  EXPECT_FALSE(input->isSuccessful());
  EXPECT_EQ(input->task_infos.getInfoMap().size(), 1);

  // Serialization
  tesseract_planning::test_suite::runSerializationPointerTest(input, "TaskComposerInputTests");

  input->reset();
  EXPECT_TRUE(input->problem != nullptr);
  EXPECT_FALSE(input->dotgraph);
  EXPECT_FALSE(input->isAborted());
  EXPECT_TRUE(input->isSuccessful());
  EXPECT_TRUE(input->task_infos.getInfoMap().empty());
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerProblemTests)  // NOLINT
{
  auto problem = std::make_unique<TaskComposerProblem>();
  EXPECT_EQ(problem->name, "unset");

  // Serialization
  tesseract_planning::test_suite::runSerializationPointerTest(problem, "TaskComposerProblemTests");

  auto problem2 = std::make_unique<TaskComposerProblem>("TaskComposerProblemTests");
  EXPECT_EQ(problem2->name, "TaskComposerProblemTests");

  auto problem3 = std::make_unique<TaskComposerProblem>(TaskComposerDataStorage(), "TaskComposerProblemTests");
  EXPECT_EQ(problem3->name, "TaskComposerProblemTests");
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerNodeTests)  // NOLINT
{
  auto node = std::make_unique<TaskComposerNode>();
  // Default
  EXPECT_EQ(node->getName(), "TaskComposerNode");
  EXPECT_EQ(node->getType(), tesseract_planning::TaskComposerNodeType::TASK);
  EXPECT_FALSE(node->getUUID().is_nil());
  EXPECT_FALSE(node->getUUIDString().empty());
  EXPECT_TRUE(node->getParentUUID().is_nil());
  EXPECT_TRUE(node->getOutboundEdges().empty());
  EXPECT_TRUE(node->getInboundEdges().empty());
  EXPECT_TRUE(node->getInputKeys().empty());
  EXPECT_TRUE(node->getOutputKeys().empty());

  // Setters
  std::string name{ "TaskComposerNodeTests" };
  std::vector<std::string> input_keys{ "I1", "I2" };
  std::vector<std::string> output_keys{ "O1", "O2" };
  node->setName(name);
  node->setInputKeys(input_keys);
  node->setOutputKeys(output_keys);
  EXPECT_EQ(node->getName(), name);
  EXPECT_EQ(node->getInputKeys(), input_keys);
  EXPECT_EQ(node->getOutputKeys(), output_keys);

  // Utils
  std::stringstream os;
  std::map<std::string, std::string> rename_input_keys{ { "I1", "I3" }, { "I2", "I4" } };
  std::map<std::string, std::string> rename_output_keys{ { "O1", "O3" }, { "O2", "O4" } };
  node->renameInputKeys(rename_input_keys);
  node->renameOutputKeys(rename_output_keys);
  EXPECT_EQ(node->getInputKeys(), std::vector<std::string>({ "I3", "I4" }));
  EXPECT_EQ(node->getOutputKeys(), std::vector<std::string>({ "O3", "O4" }));
  EXPECT_NO_THROW(node->dump(os));

  // Serialization
  tesseract_planning::test_suite::runSerializationPointerTest(node, "TaskComposerNodeTests");
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerNodeInfoTests)  // NOLINT
{
  {  // Default
    TaskComposerNodeInfo node_info;
    EXPECT_EQ(node_info.return_value, -1);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(node_info.elapsed_time, 0));
    EXPECT_TRUE(node_info.uuid.is_nil());
    EXPECT_TRUE(node_info.parent_uuid.is_nil());
    EXPECT_EQ(node_info.color, "red");
    EXPECT_FALSE(node_info.isAborted());
    EXPECT_EQ(node_info, *(node_info.clone()));

    // Serialization
    tesseract_planning::test_suite::runSerializationTest<TaskComposerNodeInfo>(node_info, "TaskComposerNodeInfoTests");
  }

  {  // Constructor
    TaskComposerNode node;
    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    TaskComposerNodeInfo node_info(node, input);
    EXPECT_EQ(node_info.return_value, -1);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(node_info.elapsed_time, 0));
    EXPECT_FALSE(node_info.uuid.is_nil());
    EXPECT_EQ(node_info.uuid, node.getUUID());
    EXPECT_EQ(node_info.parent_uuid, node.getParentUUID());
    EXPECT_EQ(node_info.color, "red");
    EXPECT_FALSE(node_info.isAborted());
    EXPECT_EQ(node_info, *(node_info.clone()));

    // Serialization
    tesseract_planning::test_suite::runSerializationTest<TaskComposerNodeInfo>(node_info, "TaskComposerNodeInfoTests");
  }

  {  // Aborted
    TaskComposerNode node;
    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    input.abort(node.getUUID());
    TaskComposerNodeInfo node_info(node, input);
    EXPECT_EQ(node_info.return_value, 0);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(node_info.elapsed_time, 0));
    EXPECT_FALSE(node_info.uuid.is_nil());
    EXPECT_EQ(node_info.uuid, node.getUUID());
    EXPECT_EQ(node_info.parent_uuid, node.getParentUUID());
    EXPECT_EQ(node_info.color, "white");
    EXPECT_TRUE(node_info.isAborted());
    EXPECT_EQ(node_info, *(node_info.clone()));

    // Serialization
    tesseract_planning::test_suite::runSerializationTest<TaskComposerNodeInfo>(node_info, "TaskComposerNodeInfoTests");
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerNodeInfoContainerTests)  // NOLINT
{
  TaskComposerNode node;
  TaskComposerInput input(std::make_unique<TaskComposerProblem>());
  auto node_info = std::make_unique<TaskComposerNodeInfo>(node, input);

  auto node_info_container = std::make_unique<TaskComposerNodeInfoContainer>();
  EXPECT_TRUE(node_info_container->getInfoMap().empty());
  node_info_container->addInfo(std::move(node_info));
  EXPECT_EQ(node_info_container->getInfoMap().size(), 1);

  // Serialization
  tesseract_planning::test_suite::runSerializationPointerTest(node_info_container,
                                                              "TaskComposerNodeInfoContainerTests");

  node_info_container->clear();
  EXPECT_TRUE(node_info_container->getInfoMap().empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
