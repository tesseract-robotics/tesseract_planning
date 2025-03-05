#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/joint_state.h>
#include <tesseract_common/utils.h>

#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_pipeline.h>
#include <tesseract_task_composer/core/task_composer_server.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_log.h>

#include <tesseract_task_composer/core/test_suite/task_composer_node_info_unit.hpp>
#include <tesseract_task_composer/core/test_suite/task_composer_serialization_utils.hpp>

#include <tesseract_task_composer/core/nodes/done_task.h>
#include <tesseract_task_composer/core/nodes/error_task.h>
#include <tesseract_task_composer/core/nodes/has_data_storage_entry_task.h>
#include <tesseract_task_composer/core/nodes/remap_task.h>
#include <tesseract_task_composer/core/nodes/start_task.h>
#include <tesseract_task_composer/core/nodes/sync_task.h>
#include <tesseract_task_composer/core/test_suite/test_task.h>

#include <tesseract_common/resource_locator.h>

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

  {  // Test Remap
    std::map<std::string, std::string> remap;
    remap[key] = "remap_" + key;

    // Test Remap Copy
    TaskComposerDataStorage remap_copy;
    remap_copy.setData(key, js);
    EXPECT_TRUE(remap_copy.hasKey(key));
    EXPECT_TRUE(remap_copy.remapData(remap, true));
    EXPECT_TRUE(remap_copy.hasKey(key));
    EXPECT_TRUE(remap_copy.hasKey("remap_" + key));
    EXPECT_EQ(remap_copy.getData(key), remap_copy.getData("remap_" + key));

    // Test Remap Move
    TaskComposerDataStorage remap_move;
    remap_move.setData(key, js);
    EXPECT_TRUE(remap_move.hasKey(key));
    EXPECT_TRUE(remap_move.remapData(remap));
    EXPECT_FALSE(remap_move.hasKey(key));
    EXPECT_TRUE(remap_move.hasKey("remap_" + key));
    EXPECT_EQ(remap_move.getData("remap_" + key).as<tesseract_common::JointState>(), js);
  }

  {  // Test Remap Failure
    std::map<std::string, std::string> remap;
    remap["does_not_exist"] = "remap_" + key;
    TaskComposerDataStorage remap_copy;
    remap_copy.setData(key, js);
    EXPECT_TRUE(remap_copy.hasKey(key));
    EXPECT_FALSE(remap_copy.remapData(remap, true));
    EXPECT_TRUE(remap_copy.hasKey(key));
    EXPECT_FALSE(remap_copy.hasKey("remap_" + key));

    // Test Remap Move
    TaskComposerDataStorage remap_move;
    remap_move.setData(key, js);
    EXPECT_TRUE(remap_move.hasKey(key));
    EXPECT_FALSE(remap_move.remapData(remap));
    EXPECT_TRUE(remap_move.hasKey(key));
    EXPECT_FALSE(remap_move.hasKey("remap_" + key));
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerContextTests)  // NOLINT
{
  test_suite::DummyTaskComposerNode node;
  auto context =
      std::make_unique<TaskComposerContext>("TaskComposerContextTests", std::make_unique<TaskComposerDataStorage>());
  EXPECT_EQ(context->name, "TaskComposerContextTests");
  EXPECT_TRUE(context->data_storage != nullptr);
  EXPECT_FALSE(context->isAborted());
  EXPECT_TRUE(context->isSuccessful());
  EXPECT_TRUE(context->task_infos.getInfoMap().empty());
  context->task_infos.addInfo(TaskComposerNodeInfo(node));
  context->abort(node.getUUID());
  EXPECT_EQ(context->task_infos.getAbortingNode(), node.getUUID());
  EXPECT_TRUE(context->isAborted());
  EXPECT_FALSE(context->isSuccessful());
  EXPECT_EQ(context->task_infos.getInfoMap().size(), 1);

  // Serialization
  test_suite::runSerializationPointerTest(context, "TaskComposerContextTests");
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerLogTests)  // NOLINT
{
  tesseract_planning::TaskComposerLog log;
  log.context =
      std::make_unique<TaskComposerContext>("TaskComposerLogTests", std::make_unique<TaskComposerDataStorage>());

  // Serialization
  test_suite::runSerializationTest(log, "TaskComposerLogTests");
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerNodeInfoContainerTests)  // NOLINT
{
  test_suite::DummyTaskComposerNode node;
  TaskComposerNodeInfo node_info(node);

  auto node_info_container = std::make_unique<TaskComposerNodeInfoContainer>();
  EXPECT_TRUE(node_info_container->getAbortingNode().is_nil());
  EXPECT_TRUE(node_info_container->getInfoMap().empty());
  auto aborted_uuid = node.getUUID();
  node_info_container->addInfo(node_info);
  node_info_container->setAborted(aborted_uuid);
  EXPECT_EQ(node_info_container->getInfoMap().size(), 1);
  EXPECT_TRUE(node_info_container->getInfo(node.getUUID()).has_value());
  EXPECT_TRUE(node_info_container->getAbortingNode() == aborted_uuid);

  // Serialization
  test_suite::runSerializationPointerTest(node_info_container, "TaskComposerNodeInfoContainerTests");

  // Copy
  auto copy_node_info_container = std::make_unique<TaskComposerNodeInfoContainer>(*node_info_container);
  EXPECT_EQ(copy_node_info_container->getInfoMap().size(), 1);
  EXPECT_TRUE(copy_node_info_container->getInfo(node.getUUID()).has_value());
  EXPECT_TRUE(copy_node_info_container->getAbortingNode() == aborted_uuid);

  // Move
  auto move_node_info_container = std::make_unique<TaskComposerNodeInfoContainer>(std::move(*node_info_container));
  EXPECT_EQ(move_node_info_container->getInfoMap().size(), 1);
  EXPECT_TRUE(move_node_info_container->getInfo(node.getUUID()).has_value());
  EXPECT_TRUE(move_node_info_container->getAbortingNode() == aborted_uuid);

  move_node_info_container->clear();
  EXPECT_TRUE(move_node_info_container->getInfoMap().empty());
  EXPECT_FALSE(move_node_info_container->getInfo(node.getUUID()).has_value());
  EXPECT_TRUE(move_node_info_container->getAbortingNode().is_nil());
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerNodeTests)  // NOLINT
{
  std::stringstream os;
  auto node = std::make_unique<test_suite::DummyTaskComposerNode>();
  // Default
  EXPECT_EQ(node->getName(), "TaskComposerNode");
  EXPECT_EQ(node->getType(), TaskComposerNodeType::NODE);
  EXPECT_FALSE(node->getUUID().is_nil());
  EXPECT_FALSE(node->getUUIDString().empty());
  EXPECT_TRUE(node->getParentUUID().is_nil());
  EXPECT_TRUE(node->getOutboundEdges().empty());
  EXPECT_TRUE(node->getInboundEdges().empty());
  EXPECT_TRUE(node->getInputKeys().empty());
  EXPECT_TRUE(node->getOutputKeys().empty());
  EXPECT_FALSE(node->isConditional());
  EXPECT_NO_THROW(node->dump(os));  // NOLINT

  // Setters
  std::string name{ "TaskComposerNodeTests" };
  TaskComposerKeys input_keys;
  input_keys.add("first", "I1");
  input_keys.add("second", "I2");
  TaskComposerKeys output_keys;
  output_keys.add("first", "O1");
  output_keys.add("second", "O2");

  node->setName(name);
  node->setInputKeys(input_keys);
  node->setOutputKeys(output_keys);
  node->setConditional(true);
  EXPECT_EQ(node->getName(), name);
  EXPECT_EQ(node->getInputKeys(), input_keys);
  EXPECT_EQ(node->getOutputKeys(), output_keys);
  EXPECT_EQ(node->isConditional(), true);
  EXPECT_NO_THROW(node->dump(os));  // NOLINT

  // Utils
  std::map<std::string, std::string> rename_input_keys{ { "I1", "I3" }, { "I2", "I4" } };
  std::map<std::string, std::string> rename_output_keys{ { "O1", "O3" }, { "O2", "O4" } };
  node->renameInputKeys(rename_input_keys);
  node->renameOutputKeys(rename_output_keys);

  TaskComposerKeys check_input_keys;
  check_input_keys.add("first", "I3");
  check_input_keys.add("second", "I4");
  TaskComposerKeys check_output_keys;
  check_output_keys.add("first", "O3");
  check_output_keys.add("second", "O4");

  EXPECT_EQ(node->getInputKeys(), check_input_keys);
  EXPECT_EQ(node->getOutputKeys(), check_output_keys);
  EXPECT_NO_THROW(node->dump(os));  // NOLINT

  // Serialization
  test_suite::runSerializationPointerTest(node, "TaskComposerNodeTests");

  {
    std::string str = R"(config:)";
    YAML::Node config = YAML::Load(str);
    auto task = std::make_unique<test_suite::DummyTaskComposerNode>(
        name, TaskComposerNodeType::TASK, TaskComposerNodePorts{}, config["config"]);
    EXPECT_EQ(task->getName(), name);
    EXPECT_EQ(task->getType(), TaskComposerNodeType::TASK);
    EXPECT_TRUE(task->getInputKeys().empty());
    EXPECT_TRUE(task->getOutputKeys().empty());
    EXPECT_FALSE(task->isConditional());

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerNodeTests");
  }

  {
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    auto task = std::make_unique<test_suite::DummyTaskComposerNode>(
        name, TaskComposerNodeType::TASK, TaskComposerNodePorts{}, config["config"]);
    EXPECT_EQ(task->getName(), name);
    EXPECT_EQ(task->getType(), TaskComposerNodeType::TASK);
    EXPECT_TRUE(task->getInputKeys().empty());
    EXPECT_TRUE(task->getOutputKeys().empty());
    EXPECT_TRUE(task->isConditional());

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerNodeTests");
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerNodeInfoTests)  // NOLINT
{
  test_suite::runTaskComposerNodeInfoTest<TaskComposerNodeInfo>();
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerTaskTests)  // NOLINT
{
  std::string name = "TaskComposerTaskTests";
  {  // Not Conditional
    auto task = std::make_unique<test_suite::TestTask>(name, false);
    EXPECT_EQ(task->getName(), name);
    EXPECT_FALSE(task->isConditional());
    EXPECT_TRUE(task->getInputKeys().empty());
    EXPECT_TRUE(task->getOutputKeys().empty());

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerTaskTests");

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerTaskTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(task->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(context->task_infos.getInfoMap().at(task->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos.getInfoMap().at(task->getUUID()).status_code, 0);

    std::stringstream os;
    EXPECT_NO_THROW(task->dump(os));                                             // NOLINT
    EXPECT_NO_THROW(task->dump(os, nullptr, context->task_infos.getInfoMap()));  // NOLINT
  }

  {  // Conditional
    auto task = std::make_unique<test_suite::TestTask>(name, true);
    task->return_value = 1;
    EXPECT_EQ(task->getName(), name);
    EXPECT_TRUE(task->isConditional());
    EXPECT_TRUE(task->getInputKeys().empty());
    EXPECT_TRUE(task->getOutputKeys().empty());

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerTaskTests");

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerTaskTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(task->run(*context), 1);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(context->task_infos.getInfoMap().at(task->getUUID()).return_value, 1);
    EXPECT_EQ(context->task_infos.getInfoMap().at(task->getUUID()).status_code, 1);

    std::stringstream os;
    EXPECT_NO_THROW(task->dump(os));                                             // NOLINT
    EXPECT_NO_THROW(task->dump(os, nullptr, context->task_infos.getInfoMap()));  // NOLINT
  }

  {
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: false
                           inputs:
                             port1: input_data
                             port2: [input_data2]
                           outputs:
                             port1: output_data
                             port2: [output_data2])";
    YAML::Node config = YAML::Load(str);
    auto task = std::make_unique<test_suite::TestTask>(name, config["config"], factory);
    EXPECT_EQ(task->getName(), name);
    EXPECT_FALSE(task->isConditional());
    EXPECT_EQ(task->getInputKeys().size(), 2);
    EXPECT_EQ(task->getOutputKeys().size(), 2);
    EXPECT_EQ(task->getInputKeys().get("port1"), "input_data");
    EXPECT_EQ(task->getOutputKeys().get("port1"), "output_data");
    EXPECT_EQ(task->getInputKeys().get<std::vector<std::string>>("port2"), std::vector<std::string>{ "input_data2" });
    EXPECT_EQ(task->getOutputKeys().get<std::vector<std::string>>("port2"), std::vector<std::string>{ "output_data2" });

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerTaskTests");

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerTaskTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(task->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(context->task_infos.getInfoMap().at(task->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos.getInfoMap().at(task->getUUID()).status_code, 0);

    std::stringstream os;
    EXPECT_NO_THROW(task->dump(os));                                             // NOLINT
    EXPECT_NO_THROW(task->dump(os, nullptr, context->task_infos.getInfoMap()));  // NOLINT
  }

  {  // Failure due to exception
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             port1: [input_data]
                             port2: [input_data2]
                           outputs:
                             port1: [output_data]
                             port2: [output_data2])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<test_suite::TestTask>(name, config["config"], factory));  // NOLINT
  }

  {  // Failure due to exception
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             port1: input_data
                             port2: input_data2
                           outputs:
                             port1: output_data
                             port2: output_data2)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<test_suite::TestTask>(name, config["config"], factory));  // NOLINT
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerPipelineTests)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  std::string name = "TaskComposerPipelineTests";
  std::string name1 = "TaskComposerPipelineTests1";
  std::string name2 = "TaskComposerPipelineTests2";
  std::string name3 = "TaskComposerPipelineTests3";
  std::string name4 = "TaskComposerPipelineTests4";
  TaskComposerKeys input_keys;
  input_keys.add("port1", "input_data");
  TaskComposerKeys output_keys;
  output_keys.add("port1", "output_data");
  std::map<std::string, std::string> rename_input_keys{ { "input_data", "id" }, { "output_data", "od" } };
  std::map<std::string, std::string> rename_output_keys{ { "output_data", "od" } };
  TaskComposerKeys new_input_keys;
  new_input_keys.add("port1", "id");
  TaskComposerKeys new_output_keys;
  new_output_keys.add("port1", "od");

  {  // Not Conditional
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, false);
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task4 = std::make_unique<test_suite::TestTask>(name4, false);
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

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerPipelineTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 5);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).status_code, 0);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test1a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test1b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos.getInfoMap()));  // NOLINT
    os2.close();
  }

  {  // Conditional
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, true);
    task2->return_value = 1;
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task4 = std::make_unique<test_suite::TestTask>(name4, false);
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

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerPipelineTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(pipeline->run(*context), 1);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 4);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).return_value, 1);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).status_code, 1);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test2a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test2b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos.getInfoMap()));  // NOLINT
    os2.close();
  }

  {  // Throw exception
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, true);
    task2->return_value = 0;
    task2->throw_exception = true;
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task4 = std::make_unique<test_suite::TestTask>(name4, false);
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

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerPipelineTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 4);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).status_code, 0);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test3a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test3b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos.getInfoMap()));  // NOLINT
    os2.close();
  }

  {  // Set Abort
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, true);
    task2->return_value = 0;
    task2->set_abort = true;
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task4 = std::make_unique<test_suite::TestTask>(name4, false);
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

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerPipelineTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_FALSE(context->isSuccessful());
    EXPECT_TRUE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 4);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).status_code, 0);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test4a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test4b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos.getInfoMap()));  // NOLINT
    os2.close();
  }

  {  // Nested Pipeline Not Conditional
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, true);
    task2->return_value = 1;
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task4 = std::make_unique<test_suite::TestTask>(name4, false);
    task4->return_value = 1;
    auto pipeline1 = std::make_unique<TaskComposerPipeline>(name + "_1", false);
    boost::uuids::uuid uuid1 = pipeline1->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline1->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline1->addNode(std::move(task3));
    boost::uuids::uuid uuid4 = pipeline1->addNode(std::move(task4));
    pipeline1->addEdges(uuid1, { uuid2 });
    pipeline1->addEdges(uuid2, { uuid3, uuid4 });
    pipeline1->setTerminals({ uuid3, uuid4 });

    auto task5 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task6 = std::make_unique<test_suite::TestTask>(name2, true);
    task6->return_value = 1;
    auto task7 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task8 = std::make_unique<test_suite::TestTask>(name4, false);
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

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerPipelineTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(pipeline3->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 9);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline3->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline3->getUUID()).status_code, 1);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test5a.dot");
    EXPECT_NO_THROW(pipeline3->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test5b.dot");
    EXPECT_NO_THROW(pipeline3->dump(os2, nullptr, context->task_infos.getInfoMap()));  // NOLINT
    os2.close();
  }

  {  // Nested Pipeline Conditional
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, true);
    task2->return_value = 1;
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task4 = std::make_unique<test_suite::TestTask>(name4, false);
    task4->return_value = 1;
    auto pipeline1 = std::make_unique<TaskComposerPipeline>(name + "_1", true);
    boost::uuids::uuid uuid1 = pipeline1->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline1->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline1->addNode(std::move(task3));
    boost::uuids::uuid uuid4 = pipeline1->addNode(std::move(task4));
    pipeline1->addEdges(uuid1, { uuid2 });
    pipeline1->addEdges(uuid2, { uuid3, uuid4 });
    pipeline1->setTerminals({ uuid3, uuid4 });

    auto task5 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task6 = std::make_unique<test_suite::TestTask>(name2, true);
    task6->return_value = 1;
    auto task7 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task8 = std::make_unique<test_suite::TestTask>(name4, false);
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
    auto task11 = std::make_unique<test_suite::TestTask>(name1, false);
    boost::uuids::uuid uuid9 = pipeline3->addNode(std::move(pipeline1));
    boost::uuids::uuid uuid10 = pipeline3->addNode(std::move(pipeline2));
    boost::uuids::uuid uuid11 = pipeline3->addNode(std::move(task11));
    pipeline3->addEdges(uuid9, { uuid11, uuid10 });
    pipeline3->setTerminals({ uuid11, uuid10 });

    // Serialization
    test_suite::runSerializationPointerTest(pipeline3, "TaskComposerPipelineTests");

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerPipelineTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(pipeline3->run(*context), 1);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 9);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline3->getUUID()).return_value, 1);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline3->getUUID()).status_code, 1);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test6a.dot");
    EXPECT_NO_THROW(pipeline3->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test6b.dot");
    EXPECT_NO_THROW(pipeline3->dump(os2, nullptr, context->task_infos.getInfoMap()));  // NOLINT
    os2.close();
  }

  {  // Nested Pipeline Abort
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, true);
    task2->return_value = 1;
    task2->set_abort = true;
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task4 = std::make_unique<test_suite::TestTask>(name4, false);
    task4->return_value = 1;
    auto pipeline1 = std::make_unique<TaskComposerPipeline>(name + "_1", true);
    boost::uuids::uuid uuid1 = pipeline1->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline1->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline1->addNode(std::move(task3));
    boost::uuids::uuid uuid4 = pipeline1->addNode(std::move(task4));
    pipeline1->addEdges(uuid1, { uuid2 });
    pipeline1->addEdges(uuid2, { uuid3, uuid4 });
    pipeline1->setTerminals({ uuid3, uuid4 });

    auto task5 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task6 = std::make_unique<test_suite::TestTask>(name2, true);
    task6->return_value = 1;
    auto task7 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task8 = std::make_unique<test_suite::TestTask>(name4, false);
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
    auto task11 = std::make_unique<test_suite::TestTask>(name1, false);
    boost::uuids::uuid uuid9 = pipeline3->addNode(std::move(pipeline1));
    boost::uuids::uuid uuid10 = pipeline3->addNode(std::move(pipeline2));
    boost::uuids::uuid uuid11 = pipeline3->addNode(std::move(task11));
    pipeline3->addEdges(uuid9, { uuid11, uuid10 });
    pipeline3->setTerminals({ uuid11, uuid10 });

    // Serialization
    test_suite::runSerializationPointerTest(pipeline3, "TaskComposerPipelineTests");

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerPipelineTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(pipeline3->run(*context), 1);
    EXPECT_FALSE(context->isSuccessful());
    EXPECT_TRUE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 6);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline3->getUUID()).return_value, 1);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline3->getUUID()).status_code, 0);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test7a.dot");
    EXPECT_NO_THROW(pipeline3->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test7b.dot");
    EXPECT_NO_THROW(pipeline3->dump(os2, nullptr, context->task_infos.getInfoMap()));  // NOLINT
    os2.close();
  }

  // This section test yaml parsing

  {
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                           outputs:
                             program: output_data
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
    EXPECT_EQ(pipeline->getInputKeys().get("program"), "input_data");
    EXPECT_EQ(pipeline->getOutputKeys().get("program"), "output_data");
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
                                   inputs:
                                     program: input_data
                                   outputs:
                                     program: output_data
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

    TaskComposerPluginFactory factory(str, locator);

    std::string str2 = R"(config:
                            conditional: true
                            inputs:
                              program: input_data
                            outputs:
                              program: output_data
                            nodes:
                              StartTask:
                                task: TestPipeline
                                config:
                                  conditional: false
                                  remapping:
                                    input_data: output_data
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
    EXPECT_EQ(pipeline->getInputKeys().get("program"), "input_data");
    EXPECT_EQ(pipeline->getOutputKeys().get("program"), "output_data");
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
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, true);
    task2->return_value = 1;
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task4 = std::make_unique<test_suite::TestTask>(name4, false);
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

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerPipelineTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).status_code, -1);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test8a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test8b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos.getInfoMap()));  // NOLINT
    os2.close();
  }

  {  // No root
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, false);
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
    auto pipeline = std::make_unique<TaskComposerPipeline>(name);
    boost::uuids::uuid uuid1 = pipeline->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline->addNode(std::move(task3));
    pipeline->setTerminals({ uuid3 });
    pipeline->addEdges(uuid1, { uuid2 });
    pipeline->addEdges(uuid2, { uuid3 });
    pipeline->addEdges(uuid3, { uuid1 });

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerPipelineTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 1);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).status_code, -1);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test9a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test9b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos.getInfoMap()));  // NOLINT
    os2.close();
  }

  {  // Non conditional with multiple out edgets
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, false);
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
    auto pipeline = std::make_unique<TaskComposerPipeline>(name);
    boost::uuids::uuid uuid1 = pipeline->addNode(std::move(task1));
    boost::uuids::uuid uuid2 = pipeline->addNode(std::move(task2));
    boost::uuids::uuid uuid3 = pipeline->addNode(std::move(task3));
    pipeline->addEdges(uuid1, { uuid2 });
    pipeline->addEdges(uuid1, { uuid3 });
    pipeline->setTerminals({ uuid2, uuid3 });

    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerPipelineTests", std::make_unique<TaskComposerDataStorage>());
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos.getInfoMap().size(), 2);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos.getInfoMap().at(pipeline->getUUID()).status_code, -1);

    std::ofstream os1;
    os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_test10a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract_common::getTempPath() + "task_composer_pipeline_test10b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos.getInfoMap()));  // NOLINT
    os2.close();
  }

  {  // Set invalid terminal
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, false);
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
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
  tesseract_common::GeneralResourceLocator locator;
  std::string name{ "TaskComposerGraphTests" };
  auto graph = std::make_unique<TaskComposerGraph>(name);
  EXPECT_EQ(graph->getName(), name);
  EXPECT_EQ(graph->getType(), TaskComposerNodeType::GRAPH);
  EXPECT_EQ(graph->isConditional(), false);

  {
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: false
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

    TaskComposerPluginFactory factory(str, locator);

    std::string str2 = R"(config:
                            conditional: true
                            nodes:
                              StartTask:
                                task:
                                  conditional: false
                                  remapping:
                                    input_data: output_data
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

    TaskComposerPluginFactory factory(str, locator);

    std::string str2 = R"(config:
                            conditional: true
                            nodes:
                              StartTask:
                                task: DoesNotExist
                                config:
                                  conditional: false
                                  remapping:
                                    input_data: output_data
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

  {  // Failure multiple root nodes
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
                                   nodes:
                                     StartTask:
                                       class: StartTaskFactory
                                       config:
                                         conditional: false
                                     DuplicateTask:
                                       class: StartTaskFactory
                                       config:
                                         conditional: false
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
                                     - source: DuplicateTask
                                       destinations: [ErrorTask, DoneTask]
                                   terminals: [ErrorTask, DoneTask])";

    TaskComposerPluginFactory factory(str, locator);

    std::string str2 = R"(config:
                            conditional: true
                            nodes:
                              StartTask:
                                class: StartTaskFactory
                                config:
                                  conditional: false
                              DuplicateTask:
                                class: StartTaskFactory
                                config:
                                  conditional: false
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
                              - source: DuplicateTask
                                destinations: [ErrorTask, DoneTask]
                            terminals: [ErrorTask, DoneTask])";
    YAML::Node config = YAML::Load(str2);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
  }

  {  // Failure terminal node with outbound edges
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
                                   nodes:
                                     StartTask:
                                       class: StartTaskFactory
                                       config:
                                         conditional: false
                                     DuplicateTask:
                                       class: StartTaskFactory
                                       config:
                                         conditional: false
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
                                     - source: ErrorTask
                                       destinations: [DuplicateTask]
                                   terminals: [ErrorTask, DoneTask])";

    TaskComposerPluginFactory factory(str, locator);

    std::string str2 = R"(config:
                            conditional: true
                            nodes:
                              StartTask:
                                class: StartTaskFactory
                                config:
                                  conditional: false
                              DuplicateTask:
                                class: StartTaskFactory
                                config:
                                  conditional: false
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
                              - source: ErrorTask
                                destinations: [DuplicateTask]
                            terminals: [ErrorTask, DoneTask])";
    YAML::Node config = YAML::Load(str2);
    EXPECT_ANY_THROW(std::make_unique<TaskComposerPipeline>(name, config["config"], factory));  // NOLINT
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

  {  // Test run method
    auto context = std::make_shared<TaskComposerContext>("TaskComposerErrorTaskTests",
                                                         std::make_unique<TaskComposerDataStorage>());
    ErrorTask task;
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message, "Error");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
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

  {  // Test run method
    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerDoneTaskTests", std::make_unique<TaskComposerDataStorage>());
    DoneTask task;
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerRemapTaskTests)  // NOLINT
{
  {  // Construction
    RemapTask task;
    EXPECT_EQ(task.getName(), "RemapTask");
    EXPECT_EQ(task.isConditional(), false);
  }

  {  // Construction
    std::map<std::string, std::string> remap;
    remap["test"] = "test2";
    RemapTask task("abc", remap, false, true);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           copy: true
                           inputs:
                             keys: [test]
                           outputs:
                             keys: [test2])";
    YAML::Node config = YAML::Load(str);
    RemapTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Serialization
    std::map<std::string, std::string> remap;
    remap["test"] = "test2";
    auto task = std::make_unique<RemapTask>("abc", remap, false, true);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerRemapTaskTests");
  }

  std::string key = "joint_state";
  std::string remap_key = "remap_joint_state";
  std::vector<std::string> joint_names{ "joint_1", "joint_2" };
  Eigen::Vector2d joint_values(5, 10);
  tesseract_common::JointState js(joint_names, joint_values);
  {  // Test run method copy
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData(key, js);
    auto context = std::make_shared<TaskComposerContext>("TaskComposerRemapTaskTests", std::move(data_storage));

    std::map<std::string, std::string> remap;
    remap[key] = remap_key;

    RemapTask task("RemapTaskTest", remap, true, true);
    EXPECT_EQ(task.run(*context), 1);
    EXPECT_TRUE(context->data_storage->hasKey(key));
    EXPECT_TRUE(context->data_storage->hasKey(remap_key));
    EXPECT_EQ(context->data_storage->getData(key), context->data_storage->getData(remap_key));
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method move
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData(key, js);
    auto context = std::make_shared<TaskComposerContext>("TaskComposerRemapTaskTests", std::move(data_storage));

    std::map<std::string, std::string> remap;
    remap[key] = remap_key;

    RemapTask task("RemapTaskTest", remap, false, true);
    EXPECT_EQ(task.run(*context), 1);
    EXPECT_FALSE(context->data_storage->hasKey(key));
    EXPECT_TRUE(context->data_storage->hasKey(remap_key));
    EXPECT_EQ(context->data_storage->getData(remap_key).as<tesseract_common::JointState>(), js);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method copy with config
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData(key, js);
    auto context = std::make_shared<TaskComposerContext>("TaskComposerRemapTaskTests", std::move(data_storage));

    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           copy: true
                           inputs:
                             keys: [joint_state]
                           outputs:
                             keys: [remap_joint_state])";
    YAML::Node config = YAML::Load(str);

    RemapTask task("RemapTaskTest", config["config"], factory);
    EXPECT_EQ(task.run(*context), 1);
    EXPECT_TRUE(context->data_storage->hasKey(key));
    EXPECT_TRUE(context->data_storage->hasKey(remap_key));
    EXPECT_EQ(context->data_storage->getData(key), context->data_storage->getData(remap_key));
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method move with config
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData(key, js);
    auto context = std::make_shared<TaskComposerContext>("TaskComposerRemapTaskTests", std::move(data_storage));

    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           copy: false
                           inputs:
                             keys: [joint_state]
                           outputs:
                             keys: [remap_joint_state])";
    YAML::Node config = YAML::Load(str);

    RemapTask task("RemapTaskTest", config["config"], factory);
    EXPECT_EQ(task.run(*context), 1);
    EXPECT_FALSE(context->data_storage->hasKey(key));
    EXPECT_TRUE(context->data_storage->hasKey(remap_key));
    EXPECT_EQ(context->data_storage->getData(remap_key).as<tesseract_common::JointState>(), js);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Failures
    std::map<std::string, std::string> remap;
    EXPECT_ANY_THROW(std::make_unique<RemapTask>("abc", remap));  // NOLINT

    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           copy: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RemapTask>("abc", config["config"], factory));  // NOLINT

    str = R"(config:
               conditional: true
               inputs:
                 keys: [input_data])";
    config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RemapTask>("abc", config["config"], factory));  // NOLINT

    str = R"(config:
               conditional: true
               outputs:
                 keys: [output_data])";
    config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RemapTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Test run method copy failure
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData(key, js);
    auto context = std::make_shared<TaskComposerContext>("TaskComposerRemapTaskTests", std::move(data_storage));

    std::map<std::string, std::string> remap;
    remap["does_not_exits"] = remap_key;

    RemapTask task("RemapTaskTest", remap, true, true);
    EXPECT_EQ(task.run(*context), 0);
    EXPECT_TRUE(context->data_storage->hasKey(key));
    EXPECT_FALSE(context->data_storage->hasKey(remap_key));
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_FALSE(node_info->status_message.empty());
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method copy failure
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData(key, js);
    auto context = std::make_shared<TaskComposerContext>("TaskComposerRemapTaskTests", std::move(data_storage));

    std::map<std::string, std::string> remap;
    remap["does_not_exits"] = remap_key;

    RemapTask task("RemapTaskTest", remap, false, true);
    EXPECT_EQ(task.run(*context), 0);
    EXPECT_TRUE(context->data_storage->hasKey(key));
    EXPECT_FALSE(context->data_storage->hasKey(remap_key));
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_FALSE(node_info->status_message.empty());
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
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
    EXPECT_ANY_THROW(std::make_unique<StartTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: false
                           inputs:
                             port1: [input_data]
                           ouputs:
                             port1: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<StartTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: false
                           outputs:
                             port1: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<StartTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<StartTask>("abc");

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerStartTaskTests");
  }

  {  // Test run method
    auto context = std::make_shared<TaskComposerContext>("TaskComposerStartTaskTests",
                                                         std::make_unique<TaskComposerDataStorage>());
    StartTask task;
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerSyncTaskTests)  // NOLINT
{
  {  // Construction
    SyncTask task;
    EXPECT_EQ(task.getName(), "SyncTask");
    EXPECT_EQ(task.isConditional(), false);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: false)";
    YAML::Node config = YAML::Load(str);
    SyncTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), false);
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<SyncTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: false
                           inputs:
                             port1: [input_data]
                           ouputs:
                             port1: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<SyncTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: false
                           outputs:
                             port1: [output_data])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<SyncTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    auto task = std::make_unique<SyncTask>("abc");

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerSyncTaskTests");
  }

  {  // Test run method
    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerSyncTaskTests", std::make_unique<TaskComposerDataStorage>());
    SyncTask task;
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerHasDataStorageEntryTaskTests)  // NOLINT
{
  {  // Construction
    HasDataStorageEntryTask task;
    EXPECT_EQ(task.getName(), "HasDataStorageEntryTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    std::vector<std::string> input_keys{ "input1", "input2" };
    HasDataStorageEntryTask task("abc", input_keys);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_TRUE(task.getOutputKeys().empty());
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    std::vector<std::string> input_keys{ "input1", "input2" };
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             keys: [input1, input2])";
    YAML::Node config = YAML::Load(str);
    HasDataStorageEntryTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_TRUE(task.getOutputKeys().empty());
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction Failure
    EXPECT_ANY_THROW(std::make_unique<HasDataStorageEntryTask>("abc", std::vector<std::string>{}));  // NOLINT
  }

  {  // Construction Failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           outputs:
                             keys: [output1, output2])";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<HasDataStorageEntryTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction Failure
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<HasDataStorageEntryTask>("abc", config["config"], factory));  // NOLINT
  }

  {  // Serialization
    std::vector<std::string> input_keys{ "input1", "input2" };
    auto task = std::make_unique<HasDataStorageEntryTask>("abc", input_keys, true);

    // Serialization
    test_suite::runSerializationPointerTest(task, "TaskComposerHasDataStorageEntryTaskTests");
  }

  {  // Test run method
    auto context = std::make_shared<TaskComposerContext>("TaskComposerHasDataStorageEntryTaskTests",
                                                         std::make_unique<TaskComposerDataStorage>());

    std::vector<std::string> input_keys{ "input1", "input2" };
    HasDataStorageEntryTask task("test_run", input_keys);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message, "Missing input key: input1");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData("input1", std::vector<std::string>{});
    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerHasDataStorageEntryTaskTests", std::move(data_storage));

    std::vector<std::string> input_keys{ "input1", "input2" };
    HasDataStorageEntryTask task("test_run", input_keys);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message, "Missing input key: input2");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }

  {  // Test run method
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData("input1", std::vector<std::string>{});
    data_storage->setData("input2", std::vector<std::string>{});
    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerHasDataStorageEntryTaskTests", std::move(data_storage));

    std::vector<std::string> input_keys{ "input1", "input2" };
    HasDataStorageEntryTask task("test_run", input_keys);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos.getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->isAborted(), false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos.getAbortingNode().is_nil());
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerServerTests)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  std::string str = R"(task_composer_plugins:
                         search_paths:
                           - /usr/local/lib
                         search_libraries:
                           - tesseract_task_composer_factories
                           - tesseract_task_composer_taskflow_factories
                         executors:
                           default: TaskflowExecutor
                           plugins:
                             TaskflowExecutor:
                               class: TaskflowTaskComposerExecutorFactory
                               config:
                                 threads: 5
                         tasks:
                           plugins:
                             TestPipeline:
                               class: PipelineTaskFactory
                               config:
                                 conditional: true
                                 nodes:
                                   StartTask:
                                     class: StartTaskFactory
                                     config:
                                       conditional: false
                                   TestTask:
                                     class: TestTaskFactory
                                     config:
                                       conditional: true
                                       return_value: 1
                                       inputs:
                                         port1: input_data
                                         port2: [input_data2]
                                       outputs:
                                         port1: output_data
                                         port2: [output_data2]
                                   DoneTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                   AbortTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                 edges:
                                   - source: StartTask
                                     destinations: [TestTask]
                                   - source: TestTask
                                     destinations: [AbortTask, DoneTask]
                                 terminals: [AbortTask, DoneTask]
                             TestGraph:
                               class: GraphTaskFactory
                               config:
                                 conditional: false
                                 nodes:
                                   StartTask:
                                     class: StartTaskFactory
                                     config:
                                       conditional: false
                                   TestTask:
                                     class: TestTaskFactory
                                     config:
                                       conditional: true
                                       return_value: 1
                                       inputs:
                                         port1: input_data
                                         port2: [input_data2]
                                       outputs:
                                         port1: output_data
                                         port2: [output_data2]
                                   DoneTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                   AbortTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                 edges:
                                   - source: StartTask
                                     destinations: [TestTask]
                                   - source: TestTask
                                     destinations: [AbortTask, DoneTask]
                                 terminals: [AbortTask, DoneTask])";

  auto runTest = [](TaskComposerServer& server) {
    std::vector<std::string> tasks{ "TestPipeline", "TestGraph" };
    std::vector<std::string> executors{ "TaskflowExecutor" };
    EXPECT_TRUE(tesseract_common::isIdentical(server.getAvailableTasks(), tasks, false));
    EXPECT_TRUE(server.hasTask("TestPipeline"));
    EXPECT_TRUE(server.hasTask("TestGraph"));
    EXPECT_NO_THROW(server.getTask("TestPipeline"));   // NOLINT
    EXPECT_NO_THROW(server.getTask("TestGraph"));      // NOLINT
    EXPECT_ANY_THROW(server.getTask("DoesNotExist"));  // NOLINT
    EXPECT_TRUE(tesseract_common::isIdentical(server.getAvailableExecutors(), executors, false));
    EXPECT_TRUE(server.hasExecutor("TaskflowExecutor"));
    EXPECT_NO_THROW(server.getExecutor("TaskflowExecutor"));  // NOLINT
    EXPECT_ANY_THROW(server.getExecutor("DoesNotExist"));     // NOLINT
    EXPECT_EQ(server.getWorkerCount("TaskflowExecutor"), 5);
    EXPECT_EQ(server.getTaskCount("TaskflowExecutor"), 0);
    EXPECT_ANY_THROW(server.getWorkerCount("DoesNotExist"));  // NOLINT
    EXPECT_ANY_THROW(server.getTaskCount("DoesNotExist"));    // NOLINT

    {  // Run method using TaskComposerContext
      auto data_storage = std::make_unique<TaskComposerDataStorage>();
      auto future = server.run("TestPipeline", std::move(data_storage), false, "TaskflowExecutor");
      future->wait();

      EXPECT_EQ(future->context->isAborted(), false);
      EXPECT_EQ(future->context->isSuccessful(), true);
      EXPECT_EQ(future->context->task_infos.getInfoMap().size(), 4);
      EXPECT_TRUE(future->context->task_infos.getAbortingNode().is_nil());
    }

    {  // Run method using Pipeline
      auto data_storage = std::make_unique<TaskComposerDataStorage>();
      const auto& pipeline = server.getTask("TestPipeline");
      auto future = server.run(pipeline, std::move(data_storage), false, "TaskflowExecutor");
      future->wait();

      EXPECT_EQ(future->context->isAborted(), false);
      EXPECT_EQ(future->context->isSuccessful(), true);
      EXPECT_EQ(future->context->task_infos.getInfoMap().size(), 4);
      EXPECT_TRUE(future->context->task_infos.getAbortingNode().is_nil());
    }

    {  // Failures, executor does not exist
      auto data_storage = std::make_unique<TaskComposerDataStorage>();
      EXPECT_ANY_THROW(server.run("TestPipeline", std::move(data_storage), false, "DoesNotExist"));  // NOLINT
    }

    {  // Failures, task does not exist
      auto data_storage = std::make_unique<TaskComposerDataStorage>();
      EXPECT_ANY_THROW(server.run("DoesNotExist", std::move(data_storage), false, "TaskflowExecutor"));  // NOLINT
    }
  };

  {  // String Constructor
    TaskComposerServer server;
    server.loadConfig(str, locator);
    runTest(server);
  }

  {  // YAML::Node Constructor
    TaskComposerServer server;
    YAML::Node config = YAML::Load(str);
    server.loadConfig(config, locator);
    runTest(server);
  }

  {  // File Path Constructor
    YAML::Node config = YAML::Load(str);
    std::filesystem::path file_path{ tesseract_common::getTempPath() + "TaskComposerServerTests.yaml" };

    {
      std::ofstream fout(file_path.string());
      fout << config;
    }

    TaskComposerServer server;
    server.loadConfig(file_path, locator);
    runTest(server);
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerPipelineWithGraphChild)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  std::string str = R"(task_composer_plugins:
                         search_paths:
                           - /usr/local/lib
                         search_libraries:
                           - tesseract_task_composer_factories
                           - tesseract_task_composer_taskflow_factories
                         executors:
                           default: TaskflowExecutor
                           plugins:
                             TaskflowExecutor:
                               class: TaskflowTaskComposerExecutorFactory
                               config:
                                 threads: 5
                         tasks:
                           plugins:
                             TestPipeline:
                               class: PipelineTaskFactory
                               config:
                                 conditional: true
                                 nodes:
                                   StartTask:
                                     class: StartTaskFactory
                                     config:
                                       conditional: false
                                   TestConditionalGraphTask:
                                     task: TestGraph
                                     config:
                                       conditional: true
                                   DoneTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                   AbortTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                 edges:
                                   - source: StartTask
                                     destinations: [TestConditionalGraphTask]
                                   - source: TestConditionalGraphTask
                                     destinations: [AbortTask, DoneTask]
                                 terminals: [AbortTask, DoneTask]
                             TestGraph:
                               class: GraphTaskFactory
                               config:
                                 conditional: false
                                 nodes:
                                   StartTask:
                                     class: StartTaskFactory
                                     config:
                                       conditional: false
                                   TestTask:
                                     class: TestTaskFactory
                                     config:
                                       conditional: true
                                       return_value: 1
                                       inputs:
                                         port1: input_data
                                         port2: [input_data2]
                                       outputs:
                                         port1: output_data
                                         port2: [output_data2]
                                   DoneTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                   AbortTask:
                                     class: DoneTaskFactory
                                     config:
                                       conditional: false
                                 edges:
                                   - source: StartTask
                                     destinations: [TestTask]
                                   - source: TestTask
                                     destinations: [AbortTask, DoneTask]
                                 terminals: [AbortTask, DoneTask])";

  TaskComposerServer server;
  server.loadConfig(str, locator);

  // Run method using TaskComposerContext
  const auto& pipeline = server.getTask("TestPipeline");
  auto data_storage = std::make_unique<TaskComposerDataStorage>();
  auto future = server.run(pipeline, std::move(data_storage), false, "TaskflowExecutor");
  future->wait();

  EXPECT_EQ(future->context->isAborted(), false);
  EXPECT_EQ(future->context->isSuccessful(), true);
  EXPECT_EQ(future->context->task_infos.getInfoMap().size(), 7);
  EXPECT_TRUE(future->context->task_infos.getAbortingNode().is_nil());

  std::ofstream os1;
  os1.open(tesseract_common::getTempPath() + "task_composer_pipeline_with_conditional_child_graph_task.dot");
  EXPECT_NO_THROW(pipeline.dump(os1, nullptr, future->context->task_infos.getInfoMap()));  // NOLINT
  os1.close();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
