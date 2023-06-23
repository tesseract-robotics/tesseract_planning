#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/joint_state.h>

#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_pipeline.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <tesseract_task_composer/core/test_suite/task_composer_node_info_unit.hpp>
#include <tesseract_task_composer/core/test_suite/task_composer_serialization_utils.hpp>
#include <tesseract_task_composer/core/test_suite/task_composer_task_unit.hpp>

#include <tesseract_task_composer/core/nodes/abort_task.h>
#include <tesseract_task_composer/core/nodes/done_task.h>
#include <tesseract_task_composer/core/nodes/error_task.h>
#include <tesseract_task_composer/core/nodes/start_task.h>

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
  EXPECT_TRUE(data.getData(key).isNull());

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

  // Test Assign
  TaskComposerDataStorage assign;
  assign = data;
  EXPECT_TRUE(assign.hasKey(key));
  EXPECT_TRUE(assign.getData().size() == 1);
  EXPECT_TRUE(assign.getData(key).as<tesseract_common::JointState>() == js);

  // Test Assign Move
  TaskComposerDataStorage move_assign;
  move_assign = std::move(data);
  EXPECT_TRUE(move_assign.hasKey(key));
  EXPECT_TRUE(move_assign.getData().size() == 1);
  EXPECT_TRUE(move_assign.getData(key).as<tesseract_common::JointState>() == js);

  // Serialization
  test_suite::runSerializationTest<TaskComposerDataStorage>(move_assign, "TaskComposerDataStorageTests");

  // Test Remove
  move_assign.removeData(key);
  EXPECT_FALSE(move_assign.hasKey(key));
  EXPECT_TRUE(move_assign.getData().empty());
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
  test_suite::runSerializationPointerTest(input, "TaskComposerInputTests");

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
  test_suite::runSerializationPointerTest(problem, "TaskComposerProblemTests");

  auto problem2 = std::make_unique<TaskComposerProblem>("TaskComposerProblemTests");
  EXPECT_EQ(problem2->name, "TaskComposerProblemTests");

  auto problem3 = std::make_unique<TaskComposerProblem>(TaskComposerDataStorage(), "TaskComposerProblemTests");
  EXPECT_EQ(problem3->name, "TaskComposerProblemTests");
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
  EXPECT_TRUE(node_info_container->getInfo(node.getUUID()) != nullptr);

  // Serialization
  test_suite::runSerializationPointerTest(node_info_container, "TaskComposerNodeInfoContainerTests");

  // Copy
  auto copy_node_info_container = std::make_unique<TaskComposerNodeInfoContainer>(*node_info_container);
  EXPECT_EQ(copy_node_info_container->getInfoMap().size(), 1);
  EXPECT_TRUE(copy_node_info_container->getInfo(node.getUUID()) != nullptr);

  // Move
  auto move_node_info_container = std::make_unique<TaskComposerNodeInfoContainer>(std::move(*node_info_container));
  EXPECT_EQ(move_node_info_container->getInfoMap().size(), 1);
  EXPECT_TRUE(move_node_info_container->getInfo(node.getUUID()) != nullptr);

  move_node_info_container->clear();
  EXPECT_TRUE(move_node_info_container->getInfoMap().empty());
  EXPECT_TRUE(move_node_info_container->getInfo(node.getUUID()) == nullptr);
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerNodeTests)  // NOLINT
{
  std::stringstream os;
  auto node = std::make_unique<TaskComposerNode>();
  // Default
  EXPECT_EQ(node->getName(), "TaskComposerNode");
  EXPECT_EQ(node->getType(), TaskComposerNodeType::TASK);
  EXPECT_FALSE(node->getUUID().is_nil());
  EXPECT_FALSE(node->getUUIDString().empty());
  EXPECT_TRUE(node->getParentUUID().is_nil());
  EXPECT_TRUE(node->getOutboundEdges().empty());
  EXPECT_TRUE(node->getInboundEdges().empty());
  EXPECT_TRUE(node->getInputKeys().empty());
  EXPECT_TRUE(node->getOutputKeys().empty());
  EXPECT_FALSE(node->isConditional());
  EXPECT_NO_THROW(node->dump(os));

  // Setters
  std::string name{ "TaskComposerNodeTests" };
  std::vector<std::string> input_keys{ "I1", "I2" };
  std::vector<std::string> output_keys{ "O1", "O2" };
  node->setName(name);
  node->setInputKeys(input_keys);
  node->setOutputKeys(output_keys);
  node->setConditional(true);
  EXPECT_EQ(node->getName(), name);
  EXPECT_EQ(node->getInputKeys(), input_keys);
  EXPECT_EQ(node->getOutputKeys(), output_keys);
  EXPECT_EQ(node->isConditional(), true);
  EXPECT_NO_THROW(node->dump(os));

  // Utils
  std::map<std::string, std::string> rename_input_keys{ { "I1", "I3" }, { "I2", "I4" } };
  std::map<std::string, std::string> rename_output_keys{ { "O1", "O3" }, { "O2", "O4" } };
  node->renameInputKeys(rename_input_keys);
  node->renameOutputKeys(rename_output_keys);
  EXPECT_EQ(node->getInputKeys(), std::vector<std::string>({ "I3", "I4" }));
  EXPECT_EQ(node->getOutputKeys(), std::vector<std::string>({ "O3", "O4" }));
  EXPECT_NO_THROW(node->dump(os));

  // Serialization
  test_suite::runSerializationPointerTest(node, "TaskComposerNodeTests");

  {
    std::string str = R"(config:
                           inputs: input_data
                           outputs: output_data)";
    YAML::Node config = YAML::Load(str);
    auto task = std::make_unique<TaskComposerNode>(name, TaskComposerNodeType::TASK, config["config"]);
    EXPECT_EQ(task->getName(), name);
    EXPECT_EQ(task->getType(), TaskComposerNodeType::TASK);
    EXPECT_EQ(task->getInputKeys().size(), 1);
    EXPECT_EQ(task->getOutputKeys().size(), 1);
    EXPECT_EQ(task->getInputKeys().front(), "input_data");
    EXPECT_EQ(task->getOutputKeys().front(), "output_data");
    EXPECT_EQ(task->isConditional(), false);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerNodeTests");
  }

  {
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data)";
    YAML::Node config = YAML::Load(str);
    auto task = std::make_unique<TaskComposerNode>(name, TaskComposerNodeType::TASK, config["config"]);
    EXPECT_EQ(task->getName(), name);
    EXPECT_EQ(task->getType(), TaskComposerNodeType::TASK);
    EXPECT_EQ(task->getInputKeys().size(), 1);
    EXPECT_EQ(task->getOutputKeys().size(), 1);
    EXPECT_EQ(task->getInputKeys().front(), "input_data");
    EXPECT_EQ(task->getOutputKeys().front(), "output_data");
    EXPECT_EQ(task->isConditional(), true);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerNodeTests");
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerNodeInfoTests)  // NOLINT
{
  test_suite::runTaskComposerNodeInfoTest<TaskComposerNodeInfo>();
}

class TestTask : public TaskComposerTask
{
public:
  using TaskComposerTask::TaskComposerTask;

  bool throw_exception{ false };
  bool set_abort{ false };
  int return_value{ 0 };

  bool operator==(const TestTask& rhs) const
  {
    bool equal = true;
    equal &= (throw_exception == rhs.throw_exception);
    equal &= (set_abort == rhs.set_abort);
    equal &= (return_value == rhs.return_value);
    equal &= TaskComposerTask::operator==(rhs);
    return equal;
  }
  bool operator!=(const TestTask& rhs) const { return !operator==(rhs); }

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_NVP(throw_exception);
    ar& BOOST_SERIALIZATION_NVP(set_abort);
    ar& BOOST_SERIALIZATION_NVP(return_value);
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
  }

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerInput& input,
                                     OptionalTaskComposerExecutor /*executor*/ = std::nullopt) const
  {
    if (throw_exception)
      throw std::runtime_error("TestTask, failure");

    auto node_info = std::make_unique<TaskComposerNodeInfo>(*this, input);
    if (conditional_)
      node_info->color = (return_value == 0) ? "red" : "green";
    else
      node_info->color = "green";
    node_info->return_value = return_value;

    if (set_abort)
    {
      node_info->color = "red";
      input.abort(uuid_);
    }

    return node_info;
  }
};

#include <tesseract_common/serialization.h>
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(TestTask, "TestTask")
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(TestTask)
BOOST_CLASS_EXPORT_IMPLEMENT(TestTask)

TEST(TesseractTaskComposerCoreUnit, TaskComposerTaskTests)  // NOLINT
{
  std::string name = "TaskComposerTaskTests";
  {  // Not Conditional
    auto task = std::make_unique<TestTask>(name, false);
    EXPECT_EQ(task->getName(), name);
    EXPECT_FALSE(task->isConditional());
    EXPECT_TRUE(task->getInputKeys().empty());
    EXPECT_TRUE(task->getOutputKeys().empty());

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerTaskTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(task->run(input), 0);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(input.task_infos.getInfoMap().at(task->getUUID())->return_value, 0);

    std::stringstream os;
    EXPECT_NO_THROW(task->dump(os));
    EXPECT_NO_THROW(task->dump(os, nullptr, input.task_infos.getInfoMap()));
  }

  {  // Conditional
    auto task = std::make_unique<TestTask>(name, true);
    task->return_value = 1;
    EXPECT_EQ(task->getName(), name);
    EXPECT_TRUE(task->isConditional());
    EXPECT_TRUE(task->getInputKeys().empty());
    EXPECT_TRUE(task->getOutputKeys().empty());

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerTaskTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(task->run(input), 1);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(input.task_infos.getInfoMap().at(task->getUUID())->return_value, 1);

    std::stringstream os;
    EXPECT_NO_THROW(task->dump(os));
    EXPECT_NO_THROW(task->dump(os, nullptr, input.task_infos.getInfoMap()));
  }

  {
    std::string str = R"(config:
                           conditional: false
                           inputs: input_data
                           outputs: output_data)";
    YAML::Node config = YAML::Load(str);
    auto task = std::make_unique<TestTask>(name, config["config"]);
    EXPECT_EQ(task->getName(), name);
    EXPECT_FALSE(task->isConditional());
    EXPECT_EQ(task->getInputKeys().size(), 1);
    EXPECT_EQ(task->getOutputKeys().size(), 1);
    EXPECT_EQ(task->getInputKeys().front(), "input_data");
    EXPECT_EQ(task->getOutputKeys().front(), "output_data");

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerTaskTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(task->run(input), 0);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(input.task_infos.getInfoMap().at(task->getUUID())->return_value, 0);

    std::stringstream os;
    EXPECT_NO_THROW(task->dump(os));
    EXPECT_NO_THROW(task->dump(os, nullptr, input.task_infos.getInfoMap()));
  }

  {
    std::string str = R"(config:
                           conditional: true
                           inputs: [input_data]
                           outputs: [output_data])";
    YAML::Node config = YAML::Load(str);
    auto task = std::make_unique<TestTask>(name, config["config"]);
    task->return_value = 1;
    EXPECT_EQ(task->getName(), name);
    EXPECT_TRUE(task->isConditional());
    EXPECT_EQ(task->getInputKeys().size(), 1);
    EXPECT_EQ(task->getOutputKeys().size(), 1);
    EXPECT_EQ(task->getInputKeys().front(), "input_data");
    EXPECT_EQ(task->getOutputKeys().front(), "output_data");

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerTaskTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(task->run(input), 1);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(input.task_infos.getInfoMap().at(task->getUUID())->return_value, 1);

    std::stringstream os;
    EXPECT_NO_THROW(task->dump(os));
    EXPECT_NO_THROW(task->dump(os, nullptr, input.task_infos.getInfoMap()));
  }

  {  // Failure due to exception during run
    std::string str = R"(config:
                           conditional: true
                           inputs: [input_data]
                           outputs: [output_data])";
    YAML::Node config = YAML::Load(str);
    auto task = std::make_unique<TestTask>(name, config["config"]);
    TaskComposerInput input(std::make_unique<TaskComposerProblem>());

    task->throw_exception = true;
    EXPECT_EQ(task->run(input), 0);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(input.task_infos.getInfoMap().at(task->getUUID())->return_value, 0);
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerPipelineTests)  // NOLINT
{
  std::string name = "TaskComposerPipelineTests";
  std::string name1 = "TaskComposerPipelineTests1";
  std::string name2 = "TaskComposerPipelineTests2";
  std::string name3 = "TaskComposerPipelineTests3";
  std::string name4 = "TaskComposerPipelineTests4";
  std::vector<std::string> input_keys{ "input_data" };
  std::vector<std::string> output_keys{ "output_data" };
  std::map<std::string, std::string> rename_input_keys{ { "input_data", "id" }, { "output_data", "od" } };
  std::map<std::string, std::string> rename_output_keys{ { "output_data", "od" } };
  std::vector<std::string> new_input_keys{ "id" };
  std::vector<std::string> new_output_keys{ "od" };
  {  // Not Conditional
    auto task1 = std::make_unique<TestTask>(name1, false);
    auto task2 = std::make_unique<TestTask>(name2, false);
    auto task3 = std::make_unique<TestTask>(name3, false);
    auto task4 = std::make_unique<TestTask>(name4, false);
    task1->setInputKeys(input_keys);
    task1->setOutputKeys(output_keys);
    task2->setInputKeys(output_keys);
    task2->setOutputKeys(output_keys);
    task3->setInputKeys(output_keys);
    task3->setOutputKeys(output_keys);
    task4->setInputKeys(output_keys);
    task4->setOutputKeys(output_keys);
    auto pipeline = std::make_unique<TaskComposerPipeline>(name);
    boost::uuids::uuid uuid1 = pipeline->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline->addNode(std::move(task3));
    boost::uuids::uuid uuid4 = pipeline->addNode(std::move(task4));
    pipeline->addEdges(uuid1, { uuid2 });
    pipeline->addEdges(uuid2, { uuid3 });
    pipeline->addEdges(uuid3, { uuid4 });
    pipeline->setTerminals({ uuid4 });
    auto nodes_map = pipeline->getNodes();
    EXPECT_EQ(pipeline->getName(), name);
    EXPECT_TRUE(pipeline->isConditional());
    EXPECT_EQ(pipeline->getTerminals(), std::vector<boost::uuids::uuid>({ uuid4 }));
    EXPECT_EQ(nodes_map.at(uuid1)->getInboundEdges().size(), 0);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid2)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid2)->getInboundEdges().front(), uuid1);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().front(), uuid3);
    EXPECT_EQ(nodes_map.at(uuid3)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid3)->getInboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid3)->getOutboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid3)->getOutboundEdges().front(), uuid4);
    EXPECT_EQ(nodes_map.at(uuid4)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid4)->getInboundEdges().front(), uuid3);
    EXPECT_EQ(nodes_map.at(uuid4)->getOutboundEdges().size(), 0);
    EXPECT_EQ(nodes_map.at(uuid1)->getInputKeys(), input_keys);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutputKeys(), output_keys);
    EXPECT_EQ(nodes_map.at(uuid2)->getInputKeys(), output_keys);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutputKeys(), output_keys);
    EXPECT_EQ(nodes_map.at(uuid3)->getInputKeys(), output_keys);
    EXPECT_EQ(nodes_map.at(uuid3)->getOutputKeys(), output_keys);
    EXPECT_EQ(nodes_map.at(uuid4)->getInputKeys(), output_keys);
    EXPECT_EQ(nodes_map.at(uuid4)->getOutputKeys(), output_keys);
    pipeline->renameInputKeys(rename_input_keys);
    pipeline->renameOutputKeys(rename_output_keys);
    EXPECT_EQ(nodes_map.at(uuid1)->getInputKeys(), new_input_keys);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutputKeys(), new_output_keys);
    EXPECT_EQ(nodes_map.at(uuid2)->getInputKeys(), new_output_keys);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutputKeys(), new_output_keys);
    EXPECT_EQ(nodes_map.at(uuid3)->getInputKeys(), new_output_keys);
    EXPECT_EQ(nodes_map.at(uuid3)->getOutputKeys(), new_output_keys);
    EXPECT_EQ(nodes_map.at(uuid4)->getInputKeys(), new_output_keys);
    EXPECT_EQ(nodes_map.at(uuid4)->getOutputKeys(), new_output_keys);

    // Serialization
    test_suite::runSerializationPointerTest(pipeline, "TaskComposerPipelineTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(pipeline->run(input), 0);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 5);
    EXPECT_EQ(input.task_infos.getInfoMap().at(pipeline->getUUID())->return_value, 0);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test1a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test1b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, input.task_infos.getInfoMap()));
    os2.close();
  }

  {  // Conditional
    auto task1 = std::make_unique<TestTask>(name1, false);
    auto task2 = std::make_unique<TestTask>(name2, true);
    task2->return_value = 1;
    auto task3 = std::make_unique<TestTask>(name3, false);
    auto task4 = std::make_unique<TestTask>(name4, false);
    task4->return_value = 1;
    auto pipeline = std::make_unique<TaskComposerPipeline>(name);
    boost::uuids::uuid uuid1 = pipeline->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline->addNode(std::move(task3));
    boost::uuids::uuid uuid4 = pipeline->addNode(std::move(task4));
    pipeline->addEdges(uuid1, { uuid2 });
    pipeline->addEdges(uuid2, { uuid3, uuid4 });
    pipeline->setTerminals({ uuid3, uuid4 });
    auto nodes_map = pipeline->getNodes();
    EXPECT_EQ(pipeline->getName(), name);
    EXPECT_TRUE(pipeline->isConditional());
    EXPECT_EQ(pipeline->getTerminals(), std::vector<boost::uuids::uuid>({ uuid3, uuid4 }));
    EXPECT_EQ(nodes_map.at(uuid1)->getInboundEdges().size(), 0);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid2)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid2)->getInboundEdges().front(), uuid1);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().size(), 2);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().front(), uuid3);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().back(), uuid4);
    EXPECT_EQ(nodes_map.at(uuid3)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid3)->getInboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid3)->getOutboundEdges().size(), 0);
    EXPECT_EQ(nodes_map.at(uuid4)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid4)->getInboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid4)->getOutboundEdges().size(), 0);

    // Serialization
    test_suite::runSerializationPointerTest(pipeline, "TaskComposerPipelineTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(pipeline->run(input), 1);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 4);
    EXPECT_EQ(input.task_infos.getInfoMap().at(pipeline->getUUID())->return_value, 1);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test2a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test2b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, input.task_infos.getInfoMap()));
    os2.close();
  }

  {  // Throw exception
    auto task1 = std::make_unique<TestTask>(name1, false);
    auto task2 = std::make_unique<TestTask>(name2, true);
    task2->return_value = 0;
    task2->throw_exception = true;
    auto task3 = std::make_unique<TestTask>(name3, false);
    auto task4 = std::make_unique<TestTask>(name4, false);
    task4->return_value = 1;
    auto pipeline = std::make_unique<TaskComposerPipeline>(name);
    boost::uuids::uuid uuid1 = pipeline->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline->addNode(std::move(task3));
    boost::uuids::uuid uuid4 = pipeline->addNode(std::move(task4));
    pipeline->addEdges(uuid1, { uuid2 });
    pipeline->addEdges(uuid2, { uuid3, uuid4 });
    pipeline->setTerminals({ uuid3, uuid4 });
    auto nodes_map = pipeline->getNodes();
    EXPECT_EQ(pipeline->getName(), name);
    EXPECT_TRUE(pipeline->isConditional());
    EXPECT_EQ(pipeline->getTerminals(), std::vector<boost::uuids::uuid>({ uuid3, uuid4 }));
    EXPECT_EQ(nodes_map.at(uuid1)->getInboundEdges().size(), 0);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid2)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid2)->getInboundEdges().front(), uuid1);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().size(), 2);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().front(), uuid3);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().back(), uuid4);
    EXPECT_EQ(nodes_map.at(uuid3)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid3)->getInboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid3)->getOutboundEdges().size(), 0);
    EXPECT_EQ(nodes_map.at(uuid4)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid4)->getInboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid4)->getOutboundEdges().size(), 0);

    // Serialization
    test_suite::runSerializationPointerTest(pipeline, "TaskComposerPipelineTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(pipeline->run(input), 0);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 4);
    EXPECT_EQ(input.task_infos.getInfoMap().at(pipeline->getUUID())->return_value, 0);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test3a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test3b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, input.task_infos.getInfoMap()));
    os2.close();
  }

  {  // Set Abort
    auto task1 = std::make_unique<TestTask>(name1, false);
    auto task2 = std::make_unique<TestTask>(name2, true);
    task2->return_value = 0;
    task2->set_abort = true;
    auto task3 = std::make_unique<TestTask>(name3, false);
    auto task4 = std::make_unique<TestTask>(name4, false);
    task4->return_value = 1;
    auto pipeline = std::make_unique<TaskComposerPipeline>(name);
    boost::uuids::uuid uuid1 = pipeline->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline->addNode(std::move(task3));
    boost::uuids::uuid uuid4 = pipeline->addNode(std::move(task4));
    pipeline->addEdges(uuid1, { uuid2 });
    pipeline->addEdges(uuid2, { uuid3, uuid4 });
    pipeline->setTerminals({ uuid3, uuid4 });
    auto nodes_map = pipeline->getNodes();
    EXPECT_EQ(pipeline->getName(), name);
    EXPECT_TRUE(pipeline->isConditional());
    EXPECT_EQ(pipeline->getTerminals(), std::vector<boost::uuids::uuid>({ uuid3, uuid4 }));
    EXPECT_EQ(nodes_map.at(uuid1)->getInboundEdges().size(), 0);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid2)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid2)->getInboundEdges().front(), uuid1);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().size(), 2);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().front(), uuid3);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().back(), uuid4);
    EXPECT_EQ(nodes_map.at(uuid3)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid3)->getInboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid3)->getOutboundEdges().size(), 0);
    EXPECT_EQ(nodes_map.at(uuid4)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid4)->getInboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid4)->getOutboundEdges().size(), 0);

    // Serialization
    test_suite::runSerializationPointerTest(pipeline, "TaskComposerPipelineTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(pipeline->run(input), 0);
    EXPECT_FALSE(input.isSuccessful());
    EXPECT_TRUE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 4);
    EXPECT_EQ(input.task_infos.getInfoMap().at(pipeline->getUUID())->return_value, 0);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test4a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test4b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, input.task_infos.getInfoMap()));
    os2.close();
  }

  {  // Nested Pipeline Not Conditional
    auto task1 = std::make_unique<TestTask>(name1, false);
    auto task2 = std::make_unique<TestTask>(name2, true);
    task2->return_value = 1;
    auto task3 = std::make_unique<TestTask>(name3, false);
    auto task4 = std::make_unique<TestTask>(name4, false);
    task4->return_value = 1;
    auto pipeline1 = std::make_unique<TaskComposerPipeline>(name + "_1", false);
    boost::uuids::uuid uuid1 = pipeline1->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline1->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline1->addNode(std::move(task3));
    boost::uuids::uuid uuid4 = pipeline1->addNode(std::move(task4));
    pipeline1->addEdges(uuid1, { uuid2 });
    pipeline1->addEdges(uuid2, { uuid3, uuid4 });
    pipeline1->setTerminals({ uuid3, uuid4 });

    auto task5 = std::make_unique<TestTask>(name1, false);
    auto task6 = std::make_unique<TestTask>(name2, true);
    task6->return_value = 1;
    auto task7 = std::make_unique<TestTask>(name3, false);
    auto task8 = std::make_unique<TestTask>(name4, false);
    task8->return_value = 1;
    auto pipeline2 = std::make_unique<TaskComposerPipeline>(name + "_2", false);
    boost::uuids::uuid uuid5 = pipeline2->addNode(std::move(task5));
    boost::uuids::uuid uuid6 = pipeline2->addNode(std::move(task6));
    boost::uuids::uuid uuid7 = pipeline2->addNode(std::move(task7));
    boost::uuids::uuid uuid8 = pipeline2->addNode(std::move(task8));
    pipeline2->addEdges(uuid5, { uuid6 });
    pipeline2->addEdges(uuid6, { uuid7, uuid8 });
    pipeline2->setTerminals({ uuid7, uuid8 });

    auto pipeline3 = std::make_unique<TaskComposerPipeline>(name + "_3");
    boost::uuids::uuid uuid9 = pipeline3->addNode(std::move(pipeline1));
    boost::uuids::uuid uuid10 = pipeline3->addNode(std::move(pipeline2));
    pipeline3->addEdges(uuid9, { uuid10 });
    pipeline3->setTerminals({ uuid10 });

    // Serialization
    test_suite::runSerializationPointerTest(pipeline3, "TaskComposerPipelineTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(pipeline3->run(input), 0);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 9);
    EXPECT_EQ(input.task_infos.getInfoMap().at(pipeline3->getUUID())->return_value, 0);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test5a.dot");
    EXPECT_NO_THROW(pipeline3->dump(os1));
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test5b.dot");
    EXPECT_NO_THROW(pipeline3->dump(os2, nullptr, input.task_infos.getInfoMap()));
    os2.close();
  }

  {  // Nested Pipeline Conditional
    auto task1 = std::make_unique<TestTask>(name1, false);
    auto task2 = std::make_unique<TestTask>(name2, true);
    task2->return_value = 1;
    auto task3 = std::make_unique<TestTask>(name3, false);
    auto task4 = std::make_unique<TestTask>(name4, false);
    task4->return_value = 1;
    auto pipeline1 = std::make_unique<TaskComposerPipeline>(name + "_1", true);
    boost::uuids::uuid uuid1 = pipeline1->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline1->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline1->addNode(std::move(task3));
    boost::uuids::uuid uuid4 = pipeline1->addNode(std::move(task4));
    pipeline1->addEdges(uuid1, { uuid2 });
    pipeline1->addEdges(uuid2, { uuid3, uuid4 });
    pipeline1->setTerminals({ uuid3, uuid4 });

    auto task5 = std::make_unique<TestTask>(name1, false);
    auto task6 = std::make_unique<TestTask>(name2, true);
    task6->return_value = 1;
    auto task7 = std::make_unique<TestTask>(name3, false);
    auto task8 = std::make_unique<TestTask>(name4, false);
    task8->return_value = 1;
    auto pipeline2 = std::make_unique<TaskComposerPipeline>(name + "_2", false);
    boost::uuids::uuid uuid5 = pipeline2->addNode(std::move(task5));
    boost::uuids::uuid uuid6 = pipeline2->addNode(std::move(task6));
    boost::uuids::uuid uuid7 = pipeline2->addNode(std::move(task7));
    boost::uuids::uuid uuid8 = pipeline2->addNode(std::move(task8));
    pipeline2->addEdges(uuid5, { uuid6 });
    pipeline2->addEdges(uuid6, { uuid7, uuid8 });
    pipeline2->setTerminals({ uuid7, uuid8 });

    auto pipeline3 = std::make_unique<TaskComposerPipeline>(name + "_3");
    auto task11 = std::make_unique<TestTask>(name1, false);
    boost::uuids::uuid uuid9 = pipeline3->addNode(std::move(pipeline1));
    boost::uuids::uuid uuid10 = pipeline3->addNode(std::move(pipeline2));
    boost::uuids::uuid uuid11 = pipeline3->addNode(std::move(task11));
    pipeline3->addEdges(uuid9, { uuid11, uuid10 });
    pipeline3->setTerminals({ uuid11, uuid10 });

    // Serialization
    test_suite::runSerializationPointerTest(pipeline3, "TaskComposerPipelineTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(pipeline3->run(input), 1);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 9);
    EXPECT_EQ(input.task_infos.getInfoMap().at(pipeline3->getUUID())->return_value, 1);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test6a.dot");
    EXPECT_NO_THROW(pipeline3->dump(os1));
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test6b.dot");
    EXPECT_NO_THROW(pipeline3->dump(os2, nullptr, input.task_infos.getInfoMap()));
    os2.close();
  }

  {  // Nested Pipeline Abort
    auto task1 = std::make_unique<TestTask>(name1, false);
    auto task2 = std::make_unique<TestTask>(name2, true);
    task2->return_value = 1;
    task2->set_abort = true;
    auto task3 = std::make_unique<TestTask>(name3, false);
    auto task4 = std::make_unique<TestTask>(name4, false);
    task4->return_value = 1;
    auto pipeline1 = std::make_unique<TaskComposerPipeline>(name + "_1", true);
    boost::uuids::uuid uuid1 = pipeline1->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline1->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline1->addNode(std::move(task3));
    boost::uuids::uuid uuid4 = pipeline1->addNode(std::move(task4));
    pipeline1->addEdges(uuid1, { uuid2 });
    pipeline1->addEdges(uuid2, { uuid3, uuid4 });
    pipeline1->setTerminals({ uuid3, uuid4 });

    auto task5 = std::make_unique<TestTask>(name1, false);
    auto task6 = std::make_unique<TestTask>(name2, true);
    task6->return_value = 1;
    auto task7 = std::make_unique<TestTask>(name3, false);
    auto task8 = std::make_unique<TestTask>(name4, false);
    task8->return_value = 1;
    auto pipeline2 = std::make_unique<TaskComposerPipeline>(name + "_2", false);
    boost::uuids::uuid uuid5 = pipeline2->addNode(std::move(task5));
    boost::uuids::uuid uuid6 = pipeline2->addNode(std::move(task6));
    boost::uuids::uuid uuid7 = pipeline2->addNode(std::move(task7));
    boost::uuids::uuid uuid8 = pipeline2->addNode(std::move(task8));
    pipeline2->addEdges(uuid5, { uuid6 });
    pipeline2->addEdges(uuid6, { uuid7, uuid8 });
    pipeline2->setTerminals({ uuid7, uuid8 });

    auto pipeline3 = std::make_unique<TaskComposerPipeline>(name + "_3");
    auto task11 = std::make_unique<TestTask>(name1, false);
    boost::uuids::uuid uuid9 = pipeline3->addNode(std::move(pipeline1));
    boost::uuids::uuid uuid10 = pipeline3->addNode(std::move(pipeline2));
    boost::uuids::uuid uuid11 = pipeline3->addNode(std::move(task11));
    pipeline3->addEdges(uuid9, { uuid11, uuid10 });
    pipeline3->setTerminals({ uuid11, uuid10 });

    // Serialization
    test_suite::runSerializationPointerTest(pipeline3, "TaskComposerPipelineTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(pipeline3->run(input), 1);
    EXPECT_FALSE(input.isSuccessful());
    EXPECT_TRUE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 6);
    EXPECT_EQ(input.task_infos.getInfoMap().at(pipeline3->getUUID())->return_value, 1);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test7a.dot");
    EXPECT_NO_THROW(pipeline3->dump(os1));
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test7b.dot");
    EXPECT_NO_THROW(pipeline3->dump(os2, nullptr, input.task_infos.getInfoMap()));
    os2.close();
  }

  // This section test yaml parsing

  {
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               class: StartTaskFactory
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             - source: StartTask
                               destinations: [DoneTask]
                           terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    auto pipeline = std::make_unique<TaskComposerPipeline>(name, config["config"], factory);
    EXPECT_TRUE(pipeline->isConditional());
    EXPECT_EQ(pipeline->getTerminals().size(), 1);
    auto task1 = pipeline->getNodeByName("StartTask");
    auto task2 = pipeline->getNodeByName("DoneTask");
    EXPECT_EQ(pipeline->getNodeByName("DoestNotExist"), nullptr);
    EXPECT_EQ(pipeline->getTerminals(), std::vector<boost::uuids::uuid>({ task2->getUUID() }));
    EXPECT_EQ(task1->getInboundEdges().size(), 0);
    EXPECT_EQ(task1->getOutboundEdges().size(), 1);
    EXPECT_EQ(task1->getOutboundEdges().front(), task2->getUUID());
    EXPECT_EQ(task2->getInboundEdges().size(), 1);
    EXPECT_EQ(task2->getInboundEdges().front(), task1->getUUID());
    EXPECT_EQ(task2->getOutboundEdges().size(), 0);
  }

  {
    std::string str = R"(task_composer_plugins:
                           search_paths:
                             - /usr/local/lib
                           search_libraries:
                             - tesseract_task_composer_factories
                           tasks:
                             plugins:
                               TestPipeline:
                                 class: PipelineTaskFactory
                                 config:
                                   conditional: true
                                   inputs: input_data
                                   outputs: output_data
                                   nodes:
                                     StartTask:
                                       class: StartTaskFactory
                                       config:
                                         conditional: false
                                     DoneTask:
                                       class: DoneTaskFactory
                                       config:
                                         conditional: false
                                   edges:
                                     - source: StartTask
                                       destinations: [DoneTask]
                                   terminals: [DoneTask])";

    TaskComposerPluginFactory factory(str);

    std::string str2 = R"(config:
                            conditional: true
                            inputs: input_data
                            outputs: output_data
                            nodes:
                              StartTask:
                                task:
                                  name: TestPipeline
                                  conditional: false
                                  input_remapping:
                                    input_data: output_data
                                  output_remapping:
                                    output_data: input_data
                              DoneTask:
                                class: DoneTaskFactory
                                config:
                                  conditional: false
                              ErrorTask:
                                class: ErrorTaskFactory
                                config:
                                  conditional: false
                            edges:
                              - source: StartTask
                                destinations: [ErrorTask, DoneTask]
                            terminals: [ErrorTask, DoneTask])";
    YAML::Node config = YAML::Load(str2);
    auto pipeline = std::make_unique<TaskComposerPipeline>(name, config["config"], factory);
    EXPECT_TRUE(pipeline->isConditional());
    EXPECT_EQ(pipeline->getTerminals().size(), 2);
    auto task1 = pipeline->getNodeByName("StartTask");
    auto task2 = pipeline->getNodeByName("ErrorTask");
    auto task3 = pipeline->getNodeByName("DoneTask");
    EXPECT_EQ(pipeline->getNodeByName("DoestNotExist"), nullptr);
    EXPECT_EQ(pipeline->getTerminals(), std::vector<boost::uuids::uuid>({ task2->getUUID(), task3->getUUID() }));
    EXPECT_EQ(task1->getInboundEdges().size(), 0);
    EXPECT_EQ(task1->getOutboundEdges().size(), 2);
    EXPECT_EQ(task1->getOutboundEdges().front(), task2->getUUID());
    EXPECT_EQ(task1->getOutboundEdges().back(), task3->getUUID());
    EXPECT_EQ(task2->getInboundEdges().size(), 1);
    EXPECT_EQ(task2->getInboundEdges().front(), task1->getUUID());
    EXPECT_EQ(task2->getOutboundEdges().size(), 0);
    EXPECT_EQ(task3->getInboundEdges().size(), 1);
    EXPECT_EQ(task3->getInboundEdges().front(), task1->getUUID());
    EXPECT_EQ(task3->getOutboundEdges().size(), 0);
  }

  // This section tests failures

  {  // Missing terminals
    auto task1 = std::make_unique<TestTask>(name1, false);
    auto task2 = std::make_unique<TestTask>(name2, true);
    task2->return_value = 1;
    auto task3 = std::make_unique<TestTask>(name3, false);
    auto task4 = std::make_unique<TestTask>(name4, false);
    task4->return_value = 1;
    auto pipeline = std::make_unique<TaskComposerPipeline>(name);
    boost::uuids::uuid uuid1 = pipeline->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline->addNode(std::move(task3));
    boost::uuids::uuid uuid4 = pipeline->addNode(std::move(task4));
    pipeline->addEdges(uuid1, { uuid2 });
    pipeline->addEdges(uuid2, { uuid3, uuid4 });
    auto nodes_map = pipeline->getNodes();
    EXPECT_EQ(pipeline->getName(), name);
    EXPECT_TRUE(pipeline->isConditional());
    EXPECT_NE(pipeline->getTerminals(), std::vector<boost::uuids::uuid>({ uuid3, uuid4 }));
    EXPECT_EQ(nodes_map.at(uuid1)->getInboundEdges().size(), 0);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid2)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid2)->getInboundEdges().front(), uuid1);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().size(), 2);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().front(), uuid3);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutboundEdges().back(), uuid4);
    EXPECT_EQ(nodes_map.at(uuid3)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid3)->getInboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid3)->getOutboundEdges().size(), 0);
    EXPECT_EQ(nodes_map.at(uuid4)->getInboundEdges().size(), 1);
    EXPECT_EQ(nodes_map.at(uuid4)->getInboundEdges().front(), uuid2);
    EXPECT_EQ(nodes_map.at(uuid4)->getOutboundEdges().size(), 0);

    // Serialization
    test_suite::runSerializationPointerTest(pipeline, "TaskComposerPipelineTests");

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(pipeline->run(input), 0);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(input.task_infos.getInfoMap().at(pipeline->getUUID())->return_value, 0);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test8a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test8b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, input.task_infos.getInfoMap()));
    os2.close();
  }

  {  // No root
    auto task1 = std::make_unique<TestTask>(name1, false);
    auto task2 = std::make_unique<TestTask>(name2, false);
    auto task3 = std::make_unique<TestTask>(name3, false);
    auto pipeline = std::make_unique<TaskComposerPipeline>(name);
    boost::uuids::uuid uuid1 = pipeline->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline->addNode(std::move(task3));
    pipeline->setTerminals({ uuid3 });
    pipeline->addEdges(uuid1, { uuid2 });
    pipeline->addEdges(uuid2, { uuid3 });
    pipeline->addEdges(uuid3, { uuid1 });

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(pipeline->run(input), 0);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(input.task_infos.getInfoMap().at(pipeline->getUUID())->return_value, 0);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test9a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test9b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, input.task_infos.getInfoMap()));
    os2.close();
  }

  {  // Non conditional with multiple out edgets
    auto task1 = std::make_unique<TestTask>(name1, false);
    auto task2 = std::make_unique<TestTask>(name2, false);
    auto task3 = std::make_unique<TestTask>(name3, false);
    auto pipeline = std::make_unique<TaskComposerPipeline>(name);
    boost::uuids::uuid uuid1 = pipeline->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline->addNode(std::move(task3));
    pipeline->addEdges(uuid1, { uuid2 });
    pipeline->addEdges(uuid1, { uuid3 });
    pipeline->setTerminals({ uuid2, uuid3 });

    TaskComposerInput input(std::make_unique<TaskComposerProblem>());
    EXPECT_EQ(pipeline->run(input), 0);
    EXPECT_TRUE(input.isSuccessful());
    EXPECT_FALSE(input.isAborted());
    EXPECT_EQ(input.task_infos.getInfoMap().size(), 2);
    EXPECT_EQ(input.task_infos.getInfoMap().at(pipeline->getUUID())->return_value, 0);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test10a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test10b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, input.task_infos.getInfoMap()));
    os2.close();
  }

  {  // Set invalid terminal
    auto task1 = std::make_unique<TestTask>(name1, false);
    auto task2 = std::make_unique<TestTask>(name2, false);
    auto task3 = std::make_unique<TestTask>(name3, false);
    auto pipeline = std::make_unique<TaskComposerPipeline>(name);
    boost::uuids::uuid uuid1 = pipeline->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline->addNode(std::move(task3));
    pipeline->addEdges(uuid1, { uuid2 });
    pipeline->addEdges(uuid2, { uuid3 });
    pipeline->addEdges(uuid3, { uuid1 });
    EXPECT_ANY_THROW(pipeline->setTerminals({ uuid3 }));                 // NOLINT
    EXPECT_ANY_THROW(pipeline->setTerminals({ boost::uuids::uuid{} }));  // NOLINT
  }

  {  // Edges is not a sequence failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               class: StartTaskFactory
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             source: StartTask
                             destinations: [DoneTask]
                           terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }

  {  // Edges source is missing
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               class: StartTaskFactory
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             - destinations: [DoneTask]
                           terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }

  {  // Edges destination is missing
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               class: StartTaskFactory
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             - source: StartTask
                           terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }

  {  // Edges source node name invalid
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               class: StartTaskFactory
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             - source: DoesNotExist
                               destinations: [DoneTask]
                           terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }

  {  // Edges destination node name invalid
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               class: StartTaskFactory
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             - source: StartTask
                               destinations: [DoesNotExist]
                           terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }

  {  // terminals is missing
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               class: StartTaskFactory
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             - source: StartTask
                               destinations: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }

  {  // terminals invalid entry
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               class: StartTaskFactory
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             - source: StartTask
                               destinations: [DoneTask]
                           terminals: [DoesNotExist])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }

  {  // Node is not a map
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             - StartTask:
                                 class: DoesNotExist
                                 config:
                                   conditional: false
                             - DoneTask:
                                 class: DoneTaskFactory
                                 config:
                                   conditional: false
                           edges:
                             - source: StartTask
                               destinations: [DoneTask]
                           terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }

  {  // Node missing class or task entry
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             - source: StartTask
                               destinations: [DoneTask]
                           terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }

  {  // Node class does not exist
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               class: DoesNotExist
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             - source: StartTask
                               destinations: [DoneTask]
                           terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }
}

// Graph is mostly tested through the Pipeline tests becasue they can be run
TEST(TesseractTaskComposerCoreUnit, TaskComposerGraphTests)  // NOLINT
{
  std::string name{ "TaskComposerGraphTests" };
  auto graph = std::make_unique<TaskComposerGraph>(name);
  EXPECT_EQ(graph->getName(), name);
  EXPECT_EQ(graph->getType(), TaskComposerNodeType::GRAPH);
  EXPECT_EQ(graph->isConditional(), false);

  {
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: false
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               class: StartTaskFactory
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             - source: StartTask
                               destinations: [DoneTask]
                           terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    auto pipeline = std::make_unique<TaskComposerGraph>(name, config["config"], factory);
    EXPECT_FALSE(pipeline->isConditional());
    EXPECT_EQ(pipeline->getTerminals().size(), 1);
    auto task1 = pipeline->getNodeByName("StartTask");
    auto task2 = pipeline->getNodeByName("DoneTask");
    EXPECT_EQ(pipeline->getNodeByName("DoestNotExist"), nullptr);
    EXPECT_EQ(pipeline->getTerminals(), std::vector<boost::uuids::uuid>({ task2->getUUID() }));
    EXPECT_EQ(task1->getInboundEdges().size(), 0);
    EXPECT_EQ(task1->getOutboundEdges().size(), 1);
    EXPECT_EQ(task1->getOutboundEdges().front(), task2->getUUID());
    EXPECT_EQ(task2->getInboundEdges().size(), 1);
    EXPECT_EQ(task2->getInboundEdges().front(), task1->getUUID());
    EXPECT_EQ(task2->getOutboundEdges().size(), 0);
  }

  {  // Failure conditional graph is currently not supported
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs: input_data
                           outputs: output_data
                           nodes:
                             StartTask:
                               class: StartTaskFactory
                               config:
                                 conditional: false
                             DoneTask:
                               class: DoneTaskFactory
                               config:
                                 conditional: false
                           edges:
                             - source: StartTask
                               destinations: [DoneTask]
                           terminals: [DoneTask])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerGraph>(name, config["config"], factory));  // NOLINT
  }

  {  // Task missing name entry
    std::string str = R"(task_composer_plugins:
                           search_paths:
                             - /usr/local/lib
                           search_libraries:
                             - tesseract_task_composer_factories
                           tasks:
                             plugins:
                               TestPipeline:
                                 class: PipelineTaskFactory
                                 config:
                                   conditional: true
                                   inputs: input_data
                                   outputs: output_data
                                   nodes:
                                     StartTask:
                                       class: StartTaskFactory
                                       config:
                                         conditional: false
                                     DoneTask:
                                       class: DoneTaskFactory
                                       config:
                                         conditional: false
                                   edges:
                                     - source: StartTask
                                       destinations: [DoneTask]
                                   terminals: [DoneTask])";

    TaskComposerPluginFactory factory(str);

    std::string str2 = R"(config:
                            conditional: true
                            inputs: input_data
                            outputs: output_data
                            nodes:
                              StartTask:
                                task:
                                  conditional: false
                                  input_remapping:
                                    input_data: output_data
                                  output_remapping:
                                    output_data: input_data
                              DoneTask:
                                class: DoneTaskFactory
                                config:
                                  conditional: false
                              ErrorTask:
                                class: ErrorTaskFactory
                                config:
                                  conditional: false
                            edges:
                              - source: StartTask
                                destinations: [ErrorTask, DoneTask]
                            terminals: [ErrorTask, DoneTask])";
    YAML::Node config = YAML::Load(str2);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }

  {  // Task name does not exist
    std::string str = R"(task_composer_plugins:
                           search_paths:
                             - /usr/local/lib
                           search_libraries:
                             - tesseract_task_composer_factories
                           tasks:
                             plugins:
                               TestPipeline:
                                 class: PipelineTaskFactory
                                 config:
                                   conditional: true
                                   inputs: input_data
                                   outputs: output_data
                                   nodes:
                                     StartTask:
                                       class: StartTaskFactory
                                       config:
                                         conditional: false
                                     DoneTask:
                                       class: DoneTaskFactory
                                       config:
                                         conditional: false
                                   edges:
                                     - source: StartTask
                                       destinations: [DoneTask]
                                   terminals: [DoneTask])";

    TaskComposerPluginFactory factory(str);

    std::string str2 = R"(config:
                            conditional: true
                            inputs: input_data
                            outputs: output_data
                            nodes:
                              StartTask:
                                task:
                                  name: DoesNotExist
                                  conditional: false
                                  input_remapping:
                                    input_data: output_data
                                  output_remapping:
                                    output_data: input_data
                              DoneTask:
                                class: DoneTaskFactory
                                config:
                                  conditional: false
                              ErrorTask:
                                class: ErrorTaskFactory
                                config:
                                  conditional: false
                            edges:
                              - source: StartTask
                                destinations: [ErrorTask, DoneTask]
                            terminals: [ErrorTask, DoneTask])";
    YAML::Node config = YAML::Load(str2);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerAbortTaskTests)  // NOLINT
{
  {  // Construction
    AbortTask task;
    EXPECT_EQ(task.getName(), "AbortTask");
    EXPECT_EQ(task.isConditional(), false);
  }

  {  // Construction
    AbortTask task("abc", true);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    AbortTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Serialization
    auto task = std::make_unique<AbortTask>("abc", true);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerAbortTaskTests");
  }

  {  // Run abort test
    AbortTask task;
    test_suite::runTaskComposerTaskAbortTest(task);
  }

  {  // Test run method
    auto input = std::make_unique<TaskComposerInput>(std::make_unique<TaskComposerProblem>());
    AbortTask task;
    EXPECT_EQ(task.run(*input), 0);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->message, "Aborted");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), true);
    EXPECT_EQ(input->isSuccessful(), false);
    EXPECT_EQ(input->task_infos.getAbortingNode(), task.getUUID());
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerErrorTaskTests)  // NOLINT
{
  {  // Construction
    ErrorTask task;
    EXPECT_EQ(task.getName(), "ErrorTask");
    EXPECT_EQ(task.isConditional(), false);
  }

  {  // Construction
    ErrorTask task("abc", true);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    ErrorTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Serialization
    auto task = std::make_unique<ErrorTask>("abc", true);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerErrorTaskTests");
  }

  {  // Run abort test
    ErrorTask task;
    test_suite::runTaskComposerTaskAbortTest(task);
  }

  {  // Test run method
    auto input = std::make_unique<TaskComposerInput>(std::make_unique<TaskComposerProblem>());
    ErrorTask task;
    EXPECT_EQ(task.run(*input), 0);
    auto node_info = input->task_infos.getInfo(task.getUUID());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->message, "Error");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(input->isAborted(), true);
    EXPECT_EQ(input->isSuccessful(), false);
    EXPECT_EQ(input->task_infos.getAbortingNode(), task.getUUID());
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerDoneTaskTests)  // NOLINT
{
  {  // Construction
    DoneTask task;
    EXPECT_EQ(task.getName(), "DoneTask");
    EXPECT_EQ(task.isConditional(), false);
  }

  {  // Construction
    DoneTask task("abc", true);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    DoneTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Serialization
    auto task = std::make_unique<DoneTask>("abc", true);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerDoneTaskTests");
  }

  {  // Run abort test
    DoneTask task;
    test_suite::runTaskComposerTaskAbortTest(task);
  }

  {  // Test run method
    auto input = std::make_unique<TaskComposerInput>(std::make_unique<TaskComposerProblem>());
    DoneTask task;
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
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerStartTaskTests)  // NOLINT
{
  {  // Construction
    StartTask task;
    EXPECT_EQ(task.getName(), "StartTask");
    EXPECT_EQ(task.isConditional(), false);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: false)";
    YAML::Node config = YAML::Load(str);
    StartTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), false);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<StartTask>("abc", config["config"], factory));
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: false
                           inputs: [input_data]
                           ouputs: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<StartTask>("abc", config["config"], factory));
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: false
                           outputs: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<StartTask>("abc", config["config"], factory));
  }

  {  // Serialization
    auto task = std::make_unique<StartTask>("abc");

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerDoneTaskTests");
  }

  {  // Run abort test
    DoneTask task;
    test_suite::runTaskComposerTaskAbortTest(task);
  }

  {  // Test run method
    auto input = std::make_unique<TaskComposerInput>(std::make_unique<TaskComposerProblem>());
    StartTask task;
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
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
