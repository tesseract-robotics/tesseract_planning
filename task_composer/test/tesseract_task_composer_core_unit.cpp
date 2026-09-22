#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract/common/joint_state.h>
#include <tesseract/common/utils.h>
#include <tesseract/common/unit_test_utils.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/property_tree.h>
#include <tesseract/common/serialization.h>

#include <tesseract/task_composer/task_composer_data_storage.h>
#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/task_composer/task_composer_executor.h>
#include <tesseract/task_composer/task_composer_future.h>
#include <tesseract/task_composer/task_composer_node.h>
#include <tesseract/task_composer/task_composer_node_info.h>
#include <tesseract/task_composer/task_composer_plugin_factory_utils.h>
#include <tesseract/task_composer/task_composer_task.h>
#include <tesseract/task_composer/task_composer_pipeline.h>
#include <tesseract/task_composer/task_composer_server.h>
#include <tesseract/task_composer/task_composer_plugin_factory.h>
#include <tesseract/task_composer/task_composer_log.h>
#include <tesseract/task_composer/cereal_serialization.h>
#include <tesseract/task_composer/yaml_extensions.h>
#include <tesseract/task_composer/yaml_utils.h>

#include <tesseract/common/schema_registry.h>

#include <tesseract/task_composer/test_suite/task_composer_node_info_unit.hpp>

#include <tesseract/task_composer/nodes/done_task.h>
#include <tesseract/task_composer/nodes/error_task.h>
#include <tesseract/task_composer/nodes/for_each_task.h>
#include <tesseract/task_composer/nodes/has_data_storage_entry_task.h>
#include <tesseract/task_composer/nodes/remap_task.h>
#include <tesseract/task_composer/nodes/start_task.h>
#include <tesseract/task_composer/nodes/sync_task.h>
#include <tesseract/task_composer/test_suite/test_task.h>

using namespace tesseract::task_composer;

namespace
{
std::string getRequiredStringAttribute(const tesseract::common::PropertyTree& schema, std::string_view name)
{
  const auto attribute = schema.getAttribute(name);
  if (!attribute.has_value())
    throw std::runtime_error("Required schema attribute is missing: " + std::string(name));

  return attribute->as<std::string>();
}
}  // namespace

TEST(TesseractTaskComposerCoreUnit, TaskComposerPortMapTests)  // NOLINT
{
  TaskComposerPortMap port_map;
  EXPECT_TRUE(port_map.empty());
  port_map.set("first", "I1");
  port_map.set("second", std::vector<std::string>{ "I2" });
  EXPECT_TRUE(port_map.size() == 2);
  EXPECT_FALSE(port_map.empty());
  EXPECT_EQ(port_map.single("first"), "I1");
  EXPECT_EQ(port_map.multiple("second"), std::vector<std::string>{ "I2" });
  EXPECT_TRUE(port_map.contains("first"));
  EXPECT_TRUE(port_map.contains("second"));
  EXPECT_THROW(port_map.single("second"), std::invalid_argument);
  EXPECT_THROW(port_map.multiple("first"), std::invalid_argument);
  EXPECT_THROW(port_map.set("", "I3"), std::invalid_argument);
  EXPECT_THROW(port_map.set("third", ""), std::invalid_argument);
  EXPECT_THROW(port_map.set("third", std::vector<std::string>{}), std::invalid_argument);
  EXPECT_THROW(port_map.set("third", std::vector<std::string>{ "I3", "" }), std::invalid_argument);
  std::map<std::string, std::string> renaming;
  renaming["I1"] = "I3";
  renaming["I2"] = "I4";
  port_map.renameStorageKeys(renaming);
  EXPECT_EQ(port_map.single("first"), "I3");
  EXPECT_EQ(port_map.multiple("second"), std::vector<std::string>{ "I4" });
  port_map.erase("second");
  EXPECT_FALSE(port_map.contains("second"));

  TaskComposerPortMap copy{ port_map };
  EXPECT_TRUE(port_map == copy);
  EXPECT_FALSE(port_map != copy);
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerNodePortsTests)  // NOLINT
{
  TaskComposerNodePorts ports;
  ports.addRequiredInput("required_single")
      .addOptionalInput("optional_multiple", TaskComposerNodePorts::Cardinality::MULTIPLE)
      .addRequiredOutput("required_multiple", TaskComposerNodePorts::Cardinality::MULTIPLE)
      .addOptionalOutput("optional_single");

  EXPECT_THROW(ports.addRequiredInput(""), std::invalid_argument);
  EXPECT_THROW(ports.addOptionalInput("required_single"), std::invalid_argument);
  EXPECT_NO_THROW(ports.addRequiredOutput("required_single"));

  TaskComposerPortMap inputs;
  TaskComposerPortMap outputs;
  EXPECT_EQ(ports.validate(inputs, outputs).size(), 3);

  inputs.set("required_single", "input");
  outputs.set("required_multiple", std::vector<std::string>{ "output1", "output2" });
  outputs.set("required_single", "output");
  EXPECT_TRUE(ports.validate(inputs, outputs).empty());

  inputs.set("optional_multiple", "wrong_cardinality");
  outputs.set("optional_single", std::vector<std::string>{ "wrong_cardinality" });
  EXPECT_EQ(ports.validate(inputs, outputs).size(), 2);

  inputs.erase("optional_multiple");
  outputs.erase("optional_single");
  inputs.set("unknown", "input");
  EXPECT_EQ(ports.validate(inputs, outputs).size(), 1);
  EXPECT_EQ(ports.validate(inputs, outputs).front().path(), "inputs.unknown");
  EXPECT_THROW(ports.validateOrThrow(inputs, outputs, "TestNode"), std::runtime_error);
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerNodePortsSchemaRuntimeParityTests)  // NOLINT
{
  TaskComposerNodePorts ports;
  ports.addRequiredInput("required_single")
      .addOptionalInput("optional_multiple", TaskComposerNodePorts::Cardinality::MULTIPLE)
      .addRequiredOutput("required_multiple", TaskComposerNodePorts::Cardinality::MULTIPLE)
      .addOptionalOutput("optional_single");

  struct TestCase
  {
    const char* inputs;
    const char* outputs;
    bool valid;
  };

  const std::vector<TestCase> test_cases{
    { "{required_single: input, optional_multiple: [input1, input2]}",
      "{required_multiple: [output1, output2], optional_single: output}",
      true },
    { "{required_single: input}", "{required_multiple: [output]}", true },
    { "{}", "{required_multiple: [output]}", false },
    { "{required_single: input}", "{}", false },
    { "{required_single: input, optional_multiple: input}", "{required_multiple: [output]}", false },
    { "{required_single: input}", "{required_multiple: output}", false },
    { "{required_single: input, unknown: input}", "{required_multiple: [output]}", false },
    { "{required_single: input}", "{required_multiple: [output], unknown: output}", false },
    { "{required_single: ''}", "{required_multiple: [output]}", false },
    { "{required_single: input, optional_multiple: []}", "{required_multiple: [output]}", false },
    { "{required_single: input, optional_multiple: [input, '']}", "{required_multiple: [output]}", false }
  };

  for (const auto& test_case : test_cases)
  {
    auto input_schema = ports.inputSchema();
    auto output_schema = ports.outputSchema();
    const bool schema_valid = input_schema.applyConfig(YAML::Load(test_case.inputs)).empty() &&
                              output_schema.applyConfig(YAML::Load(test_case.outputs)).empty();

    bool runtime_valid{ false };
    try
    {
      const auto inputs = YAML::Load(test_case.inputs).as<TaskComposerPortMap>();
      const auto outputs = YAML::Load(test_case.outputs).as<TaskComposerPortMap>();
      runtime_valid = ports.validate(inputs, outputs).empty();
    }
    catch (const std::exception&)
    {
      runtime_valid = false;
    }

    EXPECT_EQ(schema_valid, test_case.valid) << "inputs: " << test_case.inputs << ", outputs: " << test_case.outputs;
    EXPECT_EQ(runtime_valid, test_case.valid) << "inputs: " << test_case.inputs << ", outputs: " << test_case.outputs;
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerFixedPortMappingsAreAtomic)  // NOLINT
{
  test_suite::TestTask task;
  const TaskComposerPortMap original_inputs = task.getInputPortMappings();
  const TaskComposerPortMap original_outputs = task.getOutputPortMappings();

  TaskComposerPortMap invalid_inputs;
  invalid_inputs.set(test_suite::TestTask::INOUT_PORT1_PORT, "replacement_input");

  EXPECT_THROW(task.setPortMappings(invalid_inputs, original_outputs), std::runtime_error);
  EXPECT_EQ(task.getInputPortMappings(), original_inputs);
  EXPECT_EQ(task.getOutputPortMappings(), original_outputs);
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerPortSerializationTests)  // NOLINT
{
  TaskComposerPortMap mappings;
  mappings.set("single", "input");
  mappings.set("multiple", std::vector<std::string>{ "input1", "input2" });

  const std::string mappings_xml = tesseract::common::Serialization::toArchiveStringXML(mappings);
  EXPECT_NE(mappings_xml.find("<port_mappings"), std::string::npos);
  EXPECT_EQ(tesseract::common::Serialization::fromArchiveStringXML<TaskComposerPortMap>(mappings_xml), mappings);

  TaskComposerNodePorts ports;
  ports.addRequiredInput("required_input")
      .addOptionalInput("optional_input", TaskComposerNodePorts::Cardinality::MULTIPLE)
      .addRequiredOutput("required_output", TaskComposerNodePorts::Cardinality::MULTIPLE)
      .addOptionalOutput("optional_output");

  const std::string ports_xml = tesseract::common::Serialization::toArchiveStringXML(ports);
  EXPECT_NE(ports_xml.find("<input_ports"), std::string::npos);
  EXPECT_NE(ports_xml.find("<output_ports"), std::string::npos);
  EXPECT_NE(ports_xml.find("<cardinality"), std::string::npos);
  EXPECT_NE(ports_xml.find("<requirement"), std::string::npos);
  EXPECT_EQ(tesseract::common::Serialization::fromArchiveStringXML<TaskComposerNodePorts>(ports_xml), ports);
  EXPECT_EQ(tesseract::common::Serialization::fromArchiveBinaryData<TaskComposerNodePorts>(
                tesseract::common::Serialization::toArchiveBinaryData(ports)),
            ports);
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerPortMapSchemaTests)  // NOLINT
{
  {
    auto schema = YAML::convert<TaskComposerPortMap>::schema();
    EXPECT_TRUE(schema.applyConfig(YAML::Load("single: input\nmultiple: [input1, input2]")).empty());

    const YAML::Node output = schema.toYAML();
    EXPECT_EQ(output["single"].as<std::string>(), "input");
    EXPECT_EQ(output["multiple"].as<std::vector<std::string>>(), (std::vector<std::string>{ "input1", "input2" }));
  }

  {
    auto schema = YAML::convert<TaskComposerPortMap>::schema();
    const auto errors = schema.applyConfig(YAML::Load("invalid: { nested: value }"));
    EXPECT_FALSE(errors.empty());
    EXPECT_TRUE(std::any_of(errors.cbegin(), errors.cend(), [](const std::string& error) {
      return error.find("invalid") != std::string::npos && error.find("no branch matches") != std::string::npos;
    }));
  }

  {
    auto schema = YAML::convert<TaskComposerPortMap>::schema();
    const auto errors = schema.applyConfig(YAML::Load("invalid: [valid, { nested: value }]"));
    EXPECT_FALSE(errors.empty());
    EXPECT_TRUE(std::any_of(errors.cbegin(), errors.cend(), [](const std::string& error) {
      return error.find("invalid") != std::string::npos && error.find("[1]") != std::string::npos;
    }));
  }

  {
    auto schema = YAML::convert<TaskComposerPortMap>::schema();
    std::vector<std::string> errors;
    EXPECT_NO_THROW(errors = schema.applyConfig(YAML::Load("? [invalid, key]\n: value")));
    EXPECT_TRUE(std::any_of(errors.cbegin(), errors.cend(), [](const std::string& error) {
      return error.find("map key at index 0 is not a string") != std::string::npos;
    }));
  }
}

TEST(TesseractTaskComposerCoreUnit, GraphEdgeSchemaTests)  // NOLINT
{
  {
    auto schema = graphEdgeSchema();
    EXPECT_TRUE(schema.applyConfig(YAML::Load("source: start\ndestinations: finish")).empty());
    EXPECT_EQ(schema.toYAML()["destinations"].as<std::string>(), "finish");
  }

  {
    auto schema = graphEdgeSchema();
    EXPECT_TRUE(schema.applyConfig(YAML::Load("source: start\ndestinations: [middle, finish]")).empty());
    EXPECT_EQ(schema.toYAML()["destinations"].as<std::vector<std::string>>(),
              (std::vector<std::string>{ "middle", "finish" }));
  }

  {
    auto schema = graphEdgeSchema();
    const auto errors = schema.applyConfig(YAML::Load("source: start\ndestinations: { invalid: finish }"));
    EXPECT_FALSE(errors.empty());
    EXPECT_TRUE(std::any_of(errors.cbegin(), errors.cend(), [](const std::string& error) {
      return error.find("destinations") != std::string::npos && error.find("no branch matches") != std::string::npos;
    }));
  }
}

TEST(TesseractTaskComposerCoreUnit, ConstructionSchemaDefaultsTests)  // NOLINT
{
  auto task_schema = TaskComposerTask::schema(TaskComposerNodePorts{});
  EXPECT_TRUE(task_schema.applyConfig(YAML::Load("{}")).empty());
  EXPECT_FALSE(task_schema.at("conditional").as<bool>());
  EXPECT_FALSE(task_schema.at("trigger_abort").as<bool>());
  EXPECT_EQ(task_schema.find("inputs"), nullptr);
  EXPECT_EQ(task_schema.find("outputs"), nullptr);

  for (auto schema : { DoneTask::schema(), ErrorTask::schema(), StartTask::schema(), SyncTask::schema() })
  {
    EXPECT_TRUE(schema.applyConfig(YAML::Load("{}")).empty());
    EXPECT_FALSE(schema.at("conditional").as<bool>());
    EXPECT_FALSE(schema.at("trigger_abort").as<bool>());
    EXPECT_EQ(schema.find("inputs"), nullptr);
    EXPECT_EQ(schema.find("outputs"), nullptr);
  }

  auto has_data_schema = HasDataStorageEntryTask::schema();
  EXPECT_NE(has_data_schema.find("inputs"), nullptr);
  EXPECT_EQ(has_data_schema.find("outputs"), nullptr);
  EXPECT_TRUE(has_data_schema.applyConfig(YAML::Load("inputs: {storage_keys: [input]}")).empty());

  auto remap_schema = RemapTask::schema();
  EXPECT_TRUE(remap_schema.applyConfig(YAML::Load("inputs: {storage_keys: [input]}\noutputs: {storage_keys: [output]}"))
                  .empty());
  EXPECT_FALSE(remap_schema.at("copy").as<bool>());

  auto test_task_schema = test_suite::TestTask::schema();
  EXPECT_TRUE(test_task_schema
                  .applyConfig(YAML::Load("inputs: {port1: input1, port2: [input2]}\noutputs: {port1: output1, port2: "
                                          "[output2]}"))
                  .empty());
  EXPECT_FALSE(test_task_schema.at("throw_exception").as<bool>());
  EXPECT_FALSE(test_task_schema.at("set_abort").as<bool>());
  EXPECT_EQ(test_task_schema.at("return_value").as<int>(), 0);
}

TEST(TesseractTaskComposerCoreUnit, NodePortSchemaTests)  // NOLINT
{
  using namespace tesseract::common;
  using RemapTaskFactory = TaskComposerTaskFactory<RemapTask>;

  const auto node_schema = RemapTask::schema();
  ASSERT_TRUE(node_schema.at("inputs").isRequired());
  ASSERT_TRUE(node_schema.at("outputs").isRequired());
  ASSERT_TRUE(node_schema.at("inputs").at(RemapTask::INOUT_STORAGE_KEYS_PORT).isRequired());
  ASSERT_TRUE(node_schema.at("outputs").at(RemapTask::INOUT_STORAGE_KEYS_PORT).isRequired());
  EXPECT_EQ(node_schema.at("inputs").keys(), std::vector<std::string>{ RemapTask::INOUT_STORAGE_KEYS_PORT });
  EXPECT_EQ(node_schema.at("outputs").keys(), std::vector<std::string>{ RemapTask::INOUT_STORAGE_KEYS_PORT });
  EXPECT_EQ(getRequiredStringAttribute(node_schema.at("inputs").at(RemapTask::INOUT_STORAGE_KEYS_PORT),
                                       property_attribute::TYPE),
            property_type::createList(property_type::STRING));
  EXPECT_EQ(RemapTaskFactory{}.schema().at("inputs").keys(), node_schema.at("inputs").keys());

  const auto validate = [](std::string_view config) {
    auto schema = RemapTask::schema();
    return schema.applyConfig(YAML::Load(std::string(config)));
  };

  EXPECT_TRUE(validate("inputs: {storage_keys: [input]}\noutputs: {storage_keys: [output]}").empty());
  EXPECT_FALSE(validate("outputs: {storage_keys: [output]}").empty());
  EXPECT_FALSE(validate("inputs: {storage_keys: input}\noutputs: {storage_keys: [output]}").empty());
  EXPECT_FALSE(validate("inputs: {storage_keys: []}\noutputs: {storage_keys: [output]}").empty());
  EXPECT_FALSE(validate("inputs: {unknown: [input]}\noutputs: {storage_keys: [output]}").empty());

  using GraphTaskFactory = TaskComposerTaskFactory<TaskComposerGraph>;
  const auto graph_schema = GraphTaskFactory{}.schema();
  EXPECT_EQ(getRequiredStringAttribute(graph_schema.at("inputs"), property_attribute::TYPE),
            property_type::createMap(REQUIRED_STRING_OR_STRING_LIST_SCHEMA_KEY));
  EXPECT_EQ(getRequiredStringAttribute(graph_schema.at("outputs"), property_attribute::TYPE),
            property_type::createMap(REQUIRED_STRING_OR_STRING_LIST_SCHEMA_KEY));

  const auto port_mapping_schema = SchemaRegistry::instance()->get(REQUIRED_STRING_OR_STRING_LIST_SCHEMA_KEY);
  EXPECT_EQ(getRequiredStringAttribute(port_mapping_schema, property_attribute::TYPE), property_type::ONEOF);
  EXPECT_EQ(port_mapping_schema.keys(), (std::vector<std::string>{ "single", "multiple" }));
  EXPECT_EQ(getRequiredStringAttribute(port_mapping_schema.at("single"), property_attribute::TYPE),
            property_type::STRING);
  EXPECT_EQ(getRequiredStringAttribute(port_mapping_schema.at("multiple"), property_attribute::TYPE),
            property_type::createList(property_type::STRING));
}

TEST(TesseractTaskComposerCoreUnit, NodeFactoryAggregatesSchemaValidationErrors)  // NOLINT
{
  using RemapTaskFactory = TaskComposerTaskFactory<RemapTask>;

  const RemapTaskFactory factory;
  const TaskComposerPluginFactory plugin_factory;
  const YAML::Node config = YAML::Load(R"(inputs:
  storage_keys: invalid
unknown: true)");

  try
  {
    static_cast<void>(factory.create("Remap", config, plugin_factory));
    FAIL() << "Expected schema validation to fail";
  }
  catch (const tesseract::common::PropertyTreeValidationError& exception)
  {
    EXPECT_GE(exception.errors().size(), 3);
    const std::string message = exception.what();
    EXPECT_NE(message.find("inputs.storage_keys"), std::string::npos);
    EXPECT_NE(message.find("outputs"), std::string::npos);
    EXPECT_NE(message.find("unknown"), std::string::npos);
  }
}

TEST(TesseractTaskComposerCoreUnit, GraphDataFlowValidationTests)  // NOLINT
{
  TaskComposerGraph graph("GraphDataFlowValidationTests");
  const boost::uuids::uuid child_uuid = graph.addNode(std::make_unique<test_suite::TestTask>("Child", false));
  graph.setTerminals({ child_uuid });

  auto validity = graph.isValid();
  EXPECT_FALSE(validity.first);
  EXPECT_NE(validity.second.find("Child"), std::string::npos);
  EXPECT_NE(validity.second.find("input_data"), std::string::npos);

  TaskComposerPortMap graph_inputs;
  graph_inputs.set("single", "input_data");
  graph_inputs.set("multiple", std::vector<std::string>{ "input_data2" });
  graph.setPortMappings(graph_inputs, {});
  EXPECT_TRUE(graph.isValid().first);

  TaskComposerPortMap invalid_outputs;
  invalid_outputs.set("missing", "missing_output");
  graph.setPortMappings(graph_inputs, invalid_outputs);
  validity = graph.isValid();
  EXPECT_FALSE(validity.first);
  EXPECT_NE(validity.second.find("missing_output"), std::string::npos);

  TaskComposerPortMap graph_outputs;
  graph_outputs.set("single", "output_data");
  graph_outputs.set("multiple", std::vector<std::string>{ "output_data2" });
  graph.setPortMappings(graph_inputs, graph_outputs);
  EXPECT_TRUE(graph.isValid().first);

  TaskComposerPortMap invalid_override;
  invalid_override.set("single", std::vector<std::string>{ "parent_input" });
  EXPECT_THROW(graph.setOverrideInputPortMappings(invalid_override), std::runtime_error);

  TaskComposerPortMap valid_override;
  valid_override.set("single", "parent_input");
  EXPECT_NO_THROW(graph.setOverrideInputPortMappings(valid_override));
}

TEST(TesseractTaskComposerCoreUnit, NamedTaskDataFlowValidationTests)  // NOLINT
{
  const std::string plugin_config = R"(
task_composer_plugins:
  search_paths: [/usr/local/lib]
  search_libraries: [tesseract_task_composer_factories]
  tasks:
    plugins:
      NamedTestTask:
        class: TestTaskFactory
        config:
          inputs: {port1: missing_input, port2: [missing_input2]}
          outputs: {port1: output, port2: [output2]}
)";
  tesseract::common::GeneralResourceLocator locator;
  TaskComposerPluginFactory factory(plugin_config, locator);
  const YAML::Node graph_config = YAML::Load(R"(
nodes:
  Child:
    task: NamedTestTask
edges: []
terminals: [Child]
)");

  try
  {
    static_cast<void>(TaskComposerGraph("NamedDataFlowGraph", graph_config, factory));
    FAIL() << "Expected named task data-flow validation to fail";
  }
  catch (const std::runtime_error& error)
  {
    const std::string message = error.what();
    EXPECT_NE(message.find("NamedDataFlowGraph"), std::string::npos) << message;
    EXPECT_NE(message.find("Child"), std::string::npos) << message;
    EXPECT_NE(message.find("missing_input"), std::string::npos) << message;
  }
}

TEST(TesseractTaskComposerCoreUnit, ForEachTaskSchemaTests)  // NOLINT
{
  const auto validate = [](std::string_view config) {
    auto schema = ForEachTask::schema();
    try
    {
      return schema.applyConfig(YAML::Load(std::string(config)));
    }
    catch (const std::exception& e)
    {
      return std::vector<std::string>{ e.what() };
    }
  };

  EXPECT_TRUE(validate(R"(
inputs: {container: input_data}
outputs: {container: output_data}
operation:
  input_port: program
  output_port: program
  task: TestPipeline
  config:
    conditional: true
)")
                  .empty());

  for (const std::string_view config : {
           "{}",
           "operation: {output_port: program, task: TestPipeline}",
           "operation: {input_port: program, task: TestPipeline}",
           "operation: {input_port: program, output_port: program}",
           "operation: {input_port: program, output_port: program, task: TestPipeline, override: {}}",
           "operation: {input_port: program, output_port: program, task: TestPipeline, unknown: value}",
           "operation: {input_port: program, output_port: program, class: DoneTaskFactory, task: TestPipeline}",
       })
  {
    EXPECT_FALSE(validate(config).empty()) << config;
  }
}

TEST(TesseractTaskComposerCoreUnit, GraphAndPipelineSchemaTests)  // NOLINT
{
  const YAML::Node common_config = YAML::Load(R"(
namespace: custom_namespace
nodes: {}
edges: []
terminals: []
)");

  {
    auto schema = TaskComposerGraph::schema();
    EXPECT_TRUE(schema.applyConfig(common_config).empty());
    EXPECT_EQ(schema.at("namespace").as<std::string>(), "custom_namespace");
  }

  {
    YAML::Node config = YAML::Clone(common_config);
    config["conditional"] = true;
    auto schema = TaskComposerGraph::schema();
    const auto errors = schema.applyConfig(config);
    EXPECT_TRUE(schema.at("conditional").as<bool>());
    ASSERT_FALSE(errors.empty());
    EXPECT_TRUE(std::any_of(errors.cbegin(), errors.cend(), [](const std::string& error) {
      return error.find("conditional") != std::string::npos &&
             error.find("does not support conditional execution") != std::string::npos;
    }));
  }

  {
    YAML::Node config = YAML::Clone(common_config);
    config["conditional"] = true;
    auto schema = TaskComposerPipeline::schema();
    EXPECT_TRUE(schema.applyConfig(config).empty());
  }
}

TEST(TesseractTaskComposerCoreUnit, GraphSchemaReferenceTests)  // NOLINT
{
  const auto validate_destinations = [](std::string_view destinations) {
    auto schema = TaskComposerGraph::schema();
    return schema.applyConfig(YAML::Load("nodes:\n"
                                         "  start: { task: StartTask }\n"
                                         "  finish: { task: DoneTask }\n"
                                         "edges:\n"
                                         "  - source: start\n"
                                         "    destinations: " +
                                         std::string(destinations) + "\nterminals: [finish]"));
  };

  {
    const auto errors = validate_destinations("missing");
    EXPECT_TRUE(std::any_of(errors.cbegin(), errors.cend(), [](const std::string& error) {
      return error.find("destinations: 'missing' not found in nodes") != std::string::npos;
    })) << ::testing::PrintToString(errors);
  }

  {
    const auto errors = validate_destinations("[finish, missing]");
    EXPECT_TRUE(std::any_of(errors.cbegin(), errors.cend(), [](const std::string& error) {
      return error.find("destinations[1]: 'missing' not found in nodes") != std::string::npos;
    })) << ::testing::PrintToString(errors);
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerDataStorageTests)  // NOLINT
{
  std::string key{ "joint_state" };
  std::vector<tesseract::common::JointId> joint_ids{ "joint_1", "joint_2" };
  Eigen::Vector2d joint_values(5, 10);
  tesseract::common::JointState js(joint_ids, joint_values);
  TaskComposerDataStorage data("test_name");
  EXPECT_EQ(data.getName(), "test_name");
  EXPECT_FALSE(data.hasKey(key));
  EXPECT_TRUE(data.getData(key).isNull());

  // Test Add
  data.setData(key, js);
  EXPECT_TRUE(data.hasKey(key));
  EXPECT_TRUE(data.getData().size() == 1);
  EXPECT_TRUE(data.getData(key).as<tesseract::common::JointState>() == js);

  // Test Copy
  TaskComposerDataStorage copy{ data };
  EXPECT_TRUE(copy.hasKey(key));
  EXPECT_TRUE(copy.getData().size() == 1);
  EXPECT_TRUE(copy.getData(key).as<tesseract::common::JointState>() == js);

  // Test Assign
  TaskComposerDataStorage assign;
  assign = data;
  EXPECT_TRUE(assign.hasKey(key));
  EXPECT_TRUE(assign.getData().size() == 1);
  EXPECT_TRUE(assign.getData(key).as<tesseract::common::JointState>() == js);

  // Test Self Compare
  EXPECT_EQ(assign, assign);
  EXPECT_FALSE(assign != assign);
  EXPECT_TRUE(assign.hasKey(key));
  EXPECT_TRUE(assign.getData(key).as<tesseract::common::JointState>() == js);

  // Test Assign Move
  TaskComposerDataStorage move_assign;
  move_assign = std::move(data);
  EXPECT_TRUE(move_assign.hasKey(key));
  EXPECT_TRUE(move_assign.getData().size() == 1);
  EXPECT_TRUE(move_assign.getData(key).as<tesseract::common::JointState>() == js);

  // Serialization
  tesseract::common::testSerialization<TaskComposerDataStorage>(move_assign, "TaskComposerDataStorageTests");

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
    EXPECT_EQ(remap_move.getData("remap_" + key).as<tesseract::common::JointState>(), js);
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
  auto context = std::make_shared<TaskComposerContext>("TaskComposerContextTests");
  EXPECT_EQ(context->name, "TaskComposerContextTests");
  EXPECT_TRUE(context->data_storage != nullptr);
  EXPECT_FALSE(context->isAborted());
  EXPECT_TRUE(context->isSuccessful());
  EXPECT_TRUE(context->task_infos->getInfoMap().empty());
  context->task_infos->addInfo(TaskComposerNodeInfo(node));
  context->abort(node.getUUID());
  EXPECT_EQ(context->task_infos->getAbortingNode(), node.getUUID());
  EXPECT_TRUE(context->isAborted());
  EXPECT_FALSE(context->isSuccessful());
  EXPECT_EQ(context->task_infos->getInfoMap().size(), 1);

  // Serialization
  tesseract::common::testSerialization<TaskComposerContext::Ptr>(
      context,
      "TaskComposerContextTests",
      tesseract::common::testSerializationComparePtrEqual<TaskComposerContext::Ptr>);
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerLogTests)  // NOLINT
{
  TaskComposerLog log;
  log.context = std::make_shared<TaskComposerContext>("TaskComposerLogTests");

  // Serialization
  tesseract::common::testSerialization(log, "TaskComposerLogTests");
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
  tesseract::common::testSerialization<TaskComposerNodeInfoContainer::UPtr>(
      node_info_container,
      "TaskComposerNodeInfoContainerTests",
      tesseract::common::testSerializationComparePtrEqual<TaskComposerNodeInfoContainer::UPtr>);

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

  move_node_info_container->insertInfoMap(*move_node_info_container);
  EXPECT_EQ(*move_node_info_container, *move_node_info_container);
  EXPECT_FALSE(*move_node_info_container != *move_node_info_container);
  EXPECT_EQ(move_node_info_container->getInfoMap().size(), 1);

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
  EXPECT_TRUE(node->getInputPortMappings().empty());
  EXPECT_TRUE(node->getOutputPortMappings().empty());
  EXPECT_FALSE(node->isConditional());
  EXPECT_NO_THROW(node->dump(os));  // NOLINT

  // Setters
  std::string name{ "TaskComposerNodeTests" };
  TaskComposerPortMap input_port_mappings;
  input_port_mappings.set("first", "I1");
  input_port_mappings.set("second", "I2");
  TaskComposerPortMap output_port_mappings;
  output_port_mappings.set("first", "O1");
  output_port_mappings.set("second", "O2");

  node->setName(name);
  node->setPortMappings(input_port_mappings, output_port_mappings);
  node->setConditional(true);
  EXPECT_EQ(node->getName(), name);
  EXPECT_EQ(node->getInputPortMappings(), input_port_mappings);
  EXPECT_EQ(node->getOutputPortMappings(), output_port_mappings);
  EXPECT_EQ(node->isConditional(), true);
  EXPECT_NO_THROW(node->dump(os));  // NOLINT

  {
    std::string str = R"(config:)";
    YAML::Node config = YAML::Load(str);
    auto task = std::make_unique<test_suite::DummyTaskComposerNode>(
        name, TaskComposerNodeType::TASK, TaskComposerNodePorts{}, config["config"]);
    EXPECT_EQ(task->getName(), name);
    EXPECT_EQ(task->getType(), TaskComposerNodeType::TASK);
    EXPECT_TRUE(task->getInputPortMappings().empty());
    EXPECT_TRUE(task->getOutputPortMappings().empty());
    EXPECT_FALSE(task->isConditional());
  }

  {
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    auto task = std::make_unique<test_suite::DummyTaskComposerNode>(
        name, TaskComposerNodeType::TASK, TaskComposerNodePorts{}, config["config"]);
    EXPECT_EQ(task->getName(), name);
    EXPECT_EQ(task->getType(), TaskComposerNodeType::TASK);
    EXPECT_TRUE(task->getInputPortMappings().empty());
    EXPECT_TRUE(task->getOutputPortMappings().empty());
    EXPECT_TRUE(task->isConditional());
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerNodeInfoTests)  // NOLINT
{
  test_suite::runTaskComposerNodeInfoTest<TaskComposerNodeInfo>();
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerTaskTests)  // NOLINT
{
  std::string name = "TaskComposerTaskTests";
  TaskComposerDataStorage test_data;
  test_data.setData("input_data", true);
  test_data.setData("input_data2", std::vector<tesseract::common::AnyPoly>{ false });
  {  // Not Conditional
    auto task = std::make_unique<test_suite::TestTask>(name, false);
    EXPECT_EQ(task->getName(), name);
    EXPECT_FALSE(task->isConditional());
    EXPECT_FALSE(task->getInputPortMappings().empty());
    EXPECT_FALSE(task->getOutputPortMappings().empty());

    auto data = std::make_shared<TaskComposerDataStorage>(test_data);
    auto context = std::make_shared<TaskComposerContext>("TaskComposerTaskTests", data);
    EXPECT_EQ(task->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 1);
    EXPECT_EQ(context->task_infos->getInfoMap().at(task->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos->getInfoMap().at(task->getUUID()).status_code, 0);

    std::stringstream os;
    EXPECT_NO_THROW(task->dump(os));                                              // NOLINT
    EXPECT_NO_THROW(task->dump(os, nullptr, context->task_infos->getInfoMap()));  // NOLINT
  }

  {  // Conditional
    auto task = std::make_unique<test_suite::TestTask>(name, true);
    task->return_value = 1;
    EXPECT_EQ(task->getName(), name);
    EXPECT_TRUE(task->isConditional());
    EXPECT_FALSE(task->getInputPortMappings().empty());
    EXPECT_FALSE(task->getOutputPortMappings().empty());

    auto context = std::make_shared<TaskComposerContext>("TaskComposerTaskTests");
    EXPECT_EQ(task->run(*context), 1);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 1);
    EXPECT_EQ(context->task_infos->getInfoMap().at(task->getUUID()).return_value, 1);
    EXPECT_EQ(context->task_infos->getInfoMap().at(task->getUUID()).status_code, 1);

    std::stringstream os;
    EXPECT_NO_THROW(task->dump(os));                                              // NOLINT
    EXPECT_NO_THROW(task->dump(os, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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
    EXPECT_EQ(task->getInputPortMappings().size(), 2);
    EXPECT_EQ(task->getOutputPortMappings().size(), 2);
    EXPECT_EQ(task->getInputPortMappings().single("port1"), "input_data");
    EXPECT_EQ(task->getOutputPortMappings().single("port1"), "output_data");
    EXPECT_EQ(task->getInputPortMappings().multiple("port2"), std::vector<std::string>{ "input_data2" });
    EXPECT_EQ(task->getOutputPortMappings().multiple("port2"), std::vector<std::string>{ "output_data2" });

    auto context = std::make_shared<TaskComposerContext>("TaskComposerTaskTests");
    EXPECT_EQ(task->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 1);
    EXPECT_EQ(context->task_infos->getInfoMap().at(task->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos->getInfoMap().at(task->getUUID()).status_code, 0);

    std::stringstream os;
    EXPECT_NO_THROW(task->dump(os));                                              // NOLINT
    EXPECT_NO_THROW(task->dump(os, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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
  tesseract::common::GeneralResourceLocator locator;
  std::string name = "TaskComposerPipelineTests";
  std::string name1 = "TaskComposerPipelineTests1";
  std::string name2 = "TaskComposerPipelineTests2";
  std::string name3 = "TaskComposerPipelineTests3";
  std::string name4 = "TaskComposerPipelineTests4";

  TaskComposerPortMap input_port_mappings;
  input_port_mappings.set("port1", "input_data");
  input_port_mappings.set("port2", std::vector<std::string>{ "input_data2" });
  TaskComposerPortMap output_port_mappings;
  output_port_mappings.set("port1", "output_data");
  output_port_mappings.set("port2", std::vector<std::string>{ "output_data2" });

  TaskComposerDataStorage test_data;
  test_data.setData("input_data", true);
  test_data.setData("input_data2", std::vector<tesseract::common::AnyPoly>{ false });

  {  // Not Conditional
    auto task1 = std::make_unique<test_suite::TestTask>(name1, false);
    auto task2 = std::make_unique<test_suite::TestTask>(name2, false);
    auto task3 = std::make_unique<test_suite::TestTask>(name3, false);
    auto task4 = std::make_unique<test_suite::TestTask>(name4, false);
    task1->setPortMappings(input_port_mappings, output_port_mappings);
    task2->setPortMappings(output_port_mappings, output_port_mappings);
    task3->setPortMappings(output_port_mappings, output_port_mappings);
    task4->setPortMappings(output_port_mappings, output_port_mappings);
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
    EXPECT_EQ(nodes_map.at(uuid1)->getInputPortMappings(), input_port_mappings);
    EXPECT_EQ(nodes_map.at(uuid1)->getOutputPortMappings(), output_port_mappings);
    EXPECT_EQ(nodes_map.at(uuid2)->getInputPortMappings(), output_port_mappings);
    EXPECT_EQ(nodes_map.at(uuid2)->getOutputPortMappings(), output_port_mappings);
    EXPECT_EQ(nodes_map.at(uuid3)->getInputPortMappings(), output_port_mappings);
    EXPECT_EQ(nodes_map.at(uuid3)->getOutputPortMappings(), output_port_mappings);
    EXPECT_EQ(nodes_map.at(uuid4)->getInputPortMappings(), output_port_mappings);
    EXPECT_EQ(nodes_map.at(uuid4)->getOutputPortMappings(), output_port_mappings);

    auto data = std::make_shared<TaskComposerDataStorage>(test_data);
    auto context = std::make_shared<TaskComposerContext>("TaskComposerPipelineTests", std::move(data));
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 5);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).status_code, 0);

    std::ofstream os1;
    os1.open(tesseract::common::getTempPath() + "task_composer_pipeline_test1a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract::common::getTempPath() + "task_composer_pipeline_test1b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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

    auto context = std::make_shared<TaskComposerContext>("TaskComposerPipelineTests");
    EXPECT_EQ(pipeline->run(*context), 1);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 4);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).return_value, 1);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).status_code, 1);

    std::ofstream os1;
    os1.open(tesseract::common::getTempPath() + "task_composer_pipeline_test2a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract::common::getTempPath() + "task_composer_pipeline_test2b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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

    auto context = std::make_shared<TaskComposerContext>("TaskComposerPipelineTests");
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_FALSE(context->isSuccessful());
    EXPECT_TRUE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 4);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).status_code, 0);

    std::ofstream os1;
    os1.open(tesseract::common::getTempPath() + "task_composer_pipeline_test3a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract::common::getTempPath() + "task_composer_pipeline_test3b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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

    auto context = std::make_shared<TaskComposerContext>("TaskComposerPipelineTests");
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_FALSE(context->isSuccessful());
    EXPECT_TRUE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 4);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).status_code, 0);

    std::ofstream os1;
    os1.open(tesseract::common::getTempPath() + "task_composer_pipeline_test4a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract::common::getTempPath() + "task_composer_pipeline_test4b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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

    auto context = std::make_shared<TaskComposerContext>("TaskComposerPipelineTests");
    EXPECT_EQ(pipeline3->run(*context), 0);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 9);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline3->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline3->getUUID()).status_code, 1);

    std::ofstream os1;
    os1.open(tesseract::common::getTempPath() + "task_composer_pipeline_test5a.dot");
    EXPECT_NO_THROW(pipeline3->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract::common::getTempPath() + "task_composer_pipeline_test5b.dot");
    EXPECT_NO_THROW(pipeline3->dump(os2, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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

    auto context = std::make_shared<TaskComposerContext>("TaskComposerPipelineTests");
    EXPECT_EQ(pipeline3->run(*context), 1);
    EXPECT_TRUE(context->isSuccessful());
    EXPECT_FALSE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 9);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline3->getUUID()).return_value, 1);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline3->getUUID()).status_code, 1);

    std::ofstream os1;
    os1.open(tesseract::common::getTempPath() + "task_composer_pipeline_test6a.dot");
    EXPECT_NO_THROW(pipeline3->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract::common::getTempPath() + "task_composer_pipeline_test6b.dot");
    EXPECT_NO_THROW(pipeline3->dump(os2, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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

    auto context = std::make_shared<TaskComposerContext>("TaskComposerPipelineTests");
    EXPECT_EQ(pipeline3->run(*context), 1);
    EXPECT_FALSE(context->isSuccessful());
    EXPECT_TRUE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 6);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline3->getUUID()).return_value, 1);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline3->getUUID()).status_code, 0);

    std::ofstream os1;
    os1.open(tesseract::common::getTempPath() + "task_composer_pipeline_test7a.dot");
    EXPECT_NO_THROW(pipeline3->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract::common::getTempPath() + "task_composer_pipeline_test7b.dot");
    EXPECT_NO_THROW(pipeline3->dump(os2, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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
                             program: input_data
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
    EXPECT_EQ(pipeline->getInputPortMappings().single("program"), "input_data");
    EXPECT_EQ(pipeline->getOutputPortMappings().single("program"), "input_data");
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
                                     program: input_data
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
                              program: input_data
                            nodes:
                              StartTask:
                                task: TestPipeline
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
    EXPECT_EQ(pipeline->getInputPortMappings().single("program"), "input_data");
    EXPECT_EQ(pipeline->getOutputPortMappings().single("program"), "input_data");
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

    auto context = std::make_shared<TaskComposerContext>("TaskComposerPipelineTests");
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_FALSE(context->isSuccessful());
    EXPECT_TRUE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 1);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).status_code, -1);

    std::ofstream os1;
    os1.open(tesseract::common::getTempPath() + "task_composer_pipeline_test8a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract::common::getTempPath() + "task_composer_pipeline_test8b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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

    auto context = std::make_shared<TaskComposerContext>("TaskComposerPipelineTests");
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_FALSE(context->isSuccessful());
    EXPECT_TRUE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 1);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).status_code, -1);

    std::ofstream os1;
    os1.open(tesseract::common::getTempPath() + "task_composer_pipeline_test9a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract::common::getTempPath() + "task_composer_pipeline_test9b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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

    auto context = std::make_shared<TaskComposerContext>("TaskComposerPipelineTests");
    EXPECT_EQ(pipeline->run(*context), 0);
    EXPECT_FALSE(context->isSuccessful());
    EXPECT_TRUE(context->isAborted());
    EXPECT_EQ(context->task_infos->getInfoMap().size(), 2);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).return_value, 0);
    EXPECT_EQ(context->task_infos->getInfoMap().at(pipeline->getUUID()).status_code, -1);

    std::ofstream os1;
    os1.open(tesseract::common::getTempPath() + "task_composer_pipeline_test10a.dot");
    EXPECT_NO_THROW(pipeline->dump(os1));  // NOLINT
    os1.close();

    std::ofstream os2;
    os2.open(tesseract::common::getTempPath() + "task_composer_pipeline_test10b.dot");
    EXPECT_NO_THROW(pipeline->dump(os2, nullptr, context->task_infos->getInfoMap()));  // NOLINT
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
  tesseract::common::GeneralResourceLocator locator;
  std::string name{ "TaskComposerGraphTests" };
  auto graph = std::make_unique<TaskComposerGraph>(name);
  EXPECT_EQ(graph->getName(), name);
  EXPECT_EQ(graph->getType(), TaskComposerNodeType::GRAPH);
  EXPECT_EQ(graph->isConditional(), false);

  {
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           namespace: custom_namespace
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
    EXPECT_EQ(pipeline->getNamespace(), "custom_namespace");
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

  {  // Test run method
    auto context = std::make_shared<TaskComposerContext>("TaskComposerErrorTaskTests");
    ErrorTask task;
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message, "Error");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
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

  {  // Test run method
    auto context = std::make_shared<TaskComposerContext>("TaskComposerDoneTaskTests");
    DoneTask task;
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
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
                             storage_keys: [test]
                           outputs:
                             storage_keys: [test2])";
    YAML::Node config = YAML::Load(str);
    RemapTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
  }

  std::string key = "joint_state";
  std::string remap_key = "remap_joint_state";
  std::vector<tesseract::common::JointId> joint_ids{ "joint_1", "joint_2" };
  Eigen::Vector2d joint_values(5, 10);
  tesseract::common::JointState js(joint_ids, joint_values);
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
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
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
    EXPECT_EQ(context->data_storage->getData(remap_key).as<tesseract::common::JointState>(), js);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
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
                             storage_keys: [joint_state]
                           outputs:
                             storage_keys: [remap_joint_state])";
    YAML::Node config = YAML::Load(str);

    RemapTask task("RemapTaskTest", config["config"], factory);
    EXPECT_EQ(task.run(*context), 1);
    EXPECT_TRUE(context->data_storage->hasKey(key));
    EXPECT_TRUE(context->data_storage->hasKey(remap_key));
    EXPECT_EQ(context->data_storage->getData(key), context->data_storage->getData(remap_key));
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
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
                             storage_keys: [joint_state]
                           outputs:
                             storage_keys: [remap_joint_state])";
    YAML::Node config = YAML::Load(str);

    RemapTask task("RemapTaskTest", config["config"], factory);
    EXPECT_EQ(task.run(*context), 1);
    EXPECT_FALSE(context->data_storage->hasKey(key));
    EXPECT_TRUE(context->data_storage->hasKey(remap_key));
    EXPECT_EQ(context->data_storage->getData(remap_key).as<tesseract::common::JointState>(), js);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
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
                 storage_keys: [input_data])";
    config = YAML::Load(str);
    EXPECT_ANY_THROW(std::make_unique<RemapTask>("abc", config["config"], factory));  // NOLINT

    str = R"(config:
               conditional: true
               outputs:
                 storage_keys: [output_data])";
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
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_FALSE(node_info->status_message.empty());
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
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
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_FALSE(node_info->status_message.empty());
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
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

  {  // Test run method
    auto context = std::make_shared<TaskComposerContext>("TaskComposerStartTaskTests",
                                                         std::make_unique<TaskComposerDataStorage>());
    StartTask task;
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
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

  {  // Test run method
    auto context = std::make_shared<TaskComposerContext>("TaskComposerSyncTaskTests");
    SyncTask task;
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
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
    std::vector<std::string> input_storage_keys{ "input1", "input2" };
    HasDataStorageEntryTask task("abc", input_storage_keys);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_TRUE(task.getOutputPortMappings().empty());
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    TaskComposerPluginFactory factory;
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             storage_keys: [input1, input2])";
    YAML::Node config = YAML::Load(str);
    HasDataStorageEntryTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_TRUE(task.getOutputPortMappings().empty());
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
                             storage_keys: [output1, output2])";
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

  {  // Test run method
    auto context = std::make_shared<TaskComposerContext>("TaskComposerHasDataStorageEntryTaskTests",
                                                         std::make_unique<TaskComposerDataStorage>());

    std::vector<std::string> input_storage_keys{ "input1", "input2" };
    HasDataStorageEntryTask task("test_run", input_storage_keys);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message, "Missing input storage key: input1");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
  }

  {  // Test run method
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData("input1", std::vector<std::string>{});
    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerHasDataStorageEntryTaskTests", std::move(data_storage));

    std::vector<std::string> input_storage_keys{ "input1", "input2" };
    HasDataStorageEntryTask task("test_run", input_storage_keys);
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message, "Missing input storage key: input2");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
  }

  {  // Test run method
    auto data_storage = std::make_unique<TaskComposerDataStorage>();
    data_storage->setData("input1", std::vector<std::string>{});
    data_storage->setData("input2", std::vector<std::string>{});
    auto context =
        std::make_shared<TaskComposerContext>("TaskComposerHasDataStorageEntryTaskTests", std::move(data_storage));

    std::vector<std::string> input_storage_keys{ "input1", "input2" };
    HasDataStorageEntryTask task("test_run", input_storage_keys);
    EXPECT_EQ(task.run(*context), 1);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerForEachTaskTests)  // NOLINT
{
  std::string task_composer_plugins_str = R"(task_composer_plugins:
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
                                 inputs:
                                   program: input_data
                                   auxiliary: input_data2
                                 outputs:
                                   program: output_data
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

  tesseract::common::GeneralResourceLocator locator;
  TaskComposerPluginFactory factory(task_composer_plugins_str, locator);

  {  // Construction
    ForEachTask task;
    EXPECT_EQ(task.getName(), "ForEachTask");
    EXPECT_EQ(task.isConditional(), true);
  }

  {  // Construction
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             container: input_data
                           outputs:
                             container: output_data
                           operation:
                             input_port: program
                             output_port: program
                             task: TestPipeline)";
    YAML::Node config = YAML::Load(str);
    ForEachTask task("abc", config["config"], factory);
    EXPECT_EQ(task.getName(), "abc");
    EXPECT_EQ(task.isConditional(), true);
    EXPECT_EQ(task.getInputPortMappings().size(), 1);
    EXPECT_EQ(task.getInputPortMappings().single(ForEachTask::INOUT_PORT), "input_data");
    EXPECT_EQ(task.getOutputPortMappings().size(), 1);
    EXPECT_EQ(task.getOutputPortMappings().single(ForEachTask::INOUT_PORT), "output_data");
    EXPECT_EQ(task.getOutboundEdges().size(), 0);
    EXPECT_EQ(task.getInboundEdges().size(), 0);
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(TaskComposerTaskFactory<ForEachTask>{}.create("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             container: input_data)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(TaskComposerTaskFactory<ForEachTask>{}.create("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             container: input_data
                           outputs:
                             container: output_data
                           operation:
                             input_port: program
                             output_port: program)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(TaskComposerTaskFactory<ForEachTask>{}.create("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             container: input_data
                           outputs:
                             container: output_data
                           operation:
                             task: TestPipeline)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(TaskComposerTaskFactory<ForEachTask>{}.create("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             container: input_data
                           outputs:
                             container: output_data
                           operation:
                             output_port: program
                             task: TestPipeline)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(TaskComposerTaskFactory<ForEachTask>{}.create("abc", config["config"], factory));  // NOLINT
  }

  {  // Construction failure
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             container: input_data
                           outputs:
                             container: output_data
                           operation:
                             input_port: program
                             task: TestPipeline)";
    YAML::Node config = YAML::Load(str);
    EXPECT_ANY_THROW(TaskComposerTaskFactory<ForEachTask>{}.create("abc", config["config"], factory));  // NOLINT
  }

  {  // Success
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             container: input_data
                           outputs:
                             container: output_data
                           operation:
                             input_port: program
                             output_port: program
                             task: TestPipeline)";
    YAML::Node config = YAML::Load(str);
    ForEachTask task("abc", config["config"], factory);

    // Create Data Storage
    auto data = std::make_unique<TaskComposerDataStorage>();
    std::vector<tesseract::common::AnyPoly> input_data;
    input_data.emplace_back(true);
    input_data.emplace_back(false);
    data->setData("input_data", input_data);

    // Solve
    auto executor = factory.createTaskComposerExecutor(factory.getDefaultTaskComposerExecutorPlugin());
    auto context = std::make_shared<TaskComposerContext>("abc", std::move(data));
    context->dotgraph = true;
    EXPECT_EQ(task.run(*context, *executor), 1);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    std::ofstream os1;
    os1.open(tesseract::common::getTempPath() + "TaskComposerForEachTaskTests_success.dot");
    EXPECT_NO_THROW(task.dump(os1, nullptr, context->task_infos->getInfoMap()));  // NOLINT
    os1.close();

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "green");
    EXPECT_EQ(node_info->return_value, 1);
    EXPECT_EQ(node_info->status_code, 1);
    EXPECT_EQ(node_info->status_message, "Successful");
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
  }

  {  // Failure missing input data
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             container: input_data
                           outputs:
                             container: output_data
                           operation:
                             input_port: program
                             output_port: program
                             task: TestPipeline)";
    YAML::Node config = YAML::Load(str);
    ForEachTask task("abc", config["config"], factory);

    // Create Data Storage
    auto data = std::make_unique<TaskComposerDataStorage>();

    // Solve
    auto context = std::make_shared<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), true);
    EXPECT_EQ(context->isSuccessful(), false);
    EXPECT_FALSE(context->task_infos->getAbortingNode().is_nil());
  }

  {  // Failure null input data
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             container: input_data
                           outputs:
                             container: output_data
                           operation:
                             input_port: program
                             output_port: program
                             task: TestPipeline)";
    YAML::Node config = YAML::Load(str);
    ForEachTask task("abc", config["config"], factory);

    // Create data storage
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", tesseract::common::AnyPoly());

    // Solve
    auto context = std::make_shared<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, -1);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), true);
    EXPECT_EQ(context->isSuccessful(), false);
    EXPECT_FALSE(context->task_infos->getAbortingNode().is_nil());
  }

  {  // Failure input data is not composite
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             container: input_data
                           outputs:
                             container: output_data
                           operation:
                             input_port: program
                             output_port: program
                             task: TestPipeline)";
    YAML::Node config = YAML::Load(str);
    ForEachTask task("abc", config["config"], factory);

    // Create data storage
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", tesseract::common::AnyPoly(tesseract::common::JointState()));

    // Solve
    auto context = std::make_shared<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
  }

  {  // Failure input data is not composite
    std::string str = R"(config:
                           conditional: true
                           inputs:
                             container: input_data
                           outputs:
                             container: output_data
                           operation:
                             input_port: program
                             output_port: program
                             task: TestPipeline)";
    YAML::Node config = YAML::Load(str);
    ForEachTask task("abc", config["config"], factory);

    // Create data storage
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("input_data", std::vector<bool>{ true, true, false });

    // Solve
    auto context = std::make_shared<TaskComposerContext>("abc", std::move(data));
    EXPECT_EQ(task.run(*context), 0);
    auto node_info = context->task_infos->getInfo(task.getUUID());
    if (!node_info.has_value())
      throw std::runtime_error("failed");

    EXPECT_TRUE(node_info.has_value());
    EXPECT_EQ(node_info->color, "red");
    EXPECT_EQ(node_info->return_value, 0);
    EXPECT_EQ(node_info->status_code, 0);
    EXPECT_EQ(node_info->status_message.empty(), false);
    EXPECT_EQ(node_info->aborted, false);
    EXPECT_EQ(context->isAborted(), false);
    EXPECT_EQ(context->isSuccessful(), true);
    EXPECT_TRUE(context->task_infos->getAbortingNode().is_nil());
  }
}

TEST(TesseractTaskComposerCoreUnit, TaskComposerServerTests)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
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
                                 inputs:
                                   port1: input_data
                                   port2: [input_data2]
                                 outputs:
                                   port1: output_data
                                   port2: [output_data2]
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
                                 inputs:
                                   port1: input_data
                                   port2: [input_data2]
                                 outputs:
                                   port1: output_data
                                   port2: [output_data2]
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
    EXPECT_TRUE(tesseract::common::isIdentical(server.getAvailableTasks(), tasks, false));
    EXPECT_TRUE(server.hasTask("TestPipeline"));
    EXPECT_TRUE(server.hasTask("TestGraph"));
    EXPECT_NO_THROW(server.getTask("TestPipeline"));   // NOLINT
    EXPECT_NO_THROW(server.getTask("TestGraph"));      // NOLINT
    EXPECT_ANY_THROW(server.getTask("DoesNotExist"));  // NOLINT
    EXPECT_TRUE(tesseract::common::isIdentical(server.getAvailableExecutors(), executors, false));
    EXPECT_TRUE(server.hasExecutor("TaskflowExecutor"));
    EXPECT_NO_THROW(server.getExecutor("TaskflowExecutor"));  // NOLINT
    EXPECT_ANY_THROW(server.getExecutor("DoesNotExist"));     // NOLINT
    EXPECT_EQ(server.getWorkerCount("TaskflowExecutor"), 5);
    EXPECT_EQ(server.getTaskCount("TaskflowExecutor"), 0);
    EXPECT_ANY_THROW(server.getWorkerCount("DoesNotExist"));  // NOLINT
    EXPECT_ANY_THROW(server.getTaskCount("DoesNotExist"));    // NOLINT

    {  // Run method using TaskComposerContext
      auto data_storage = std::make_unique<TaskComposerDataStorage>();
      data_storage->setData("input_data", true);
      data_storage->setData("input_data2", std::vector<tesseract::common::AnyPoly>{ false });
      auto future = server.run("TestPipeline", std::move(data_storage), false, "TaskflowExecutor");
      future->wait();

      EXPECT_EQ(future->context->isAborted(), false);
      EXPECT_EQ(future->context->isSuccessful(), true);
      EXPECT_EQ(future->context->task_infos->getInfoMap().size(), 4);
      EXPECT_TRUE(future->context->task_infos->getAbortingNode().is_nil());
    }

    {  // Run method using Pipeline
      auto data_storage = std::make_unique<TaskComposerDataStorage>();
      data_storage->setData("input_data", true);
      data_storage->setData("input_data2", std::vector<tesseract::common::AnyPoly>{ false });
      const auto& pipeline = server.getTask("TestPipeline");
      auto future = server.run(pipeline, std::move(data_storage), false, "TaskflowExecutor");
      future->wait();

      EXPECT_EQ(future->context->isAborted(), false);
      EXPECT_EQ(future->context->isSuccessful(), true);
      EXPECT_EQ(future->context->task_infos->getInfoMap().size(), 4);
      EXPECT_TRUE(future->context->task_infos->getAbortingNode().is_nil());
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
    std::filesystem::path file_path{ tesseract::common::getTempPath() + "TaskComposerServerTests.yaml" };

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
  tesseract::common::GeneralResourceLocator locator;
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
                                 inputs:
                                   port1: input_data
                                   port2: [input_data2]
                                 outputs:
                                   port1: output_data
                                   port2: [output_data2]
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
                                 inputs:
                                   port1: input_data
                                   port2: [input_data2]
                                 outputs:
                                   port1: output_data
                                   port2: [output_data2]
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
  data_storage->setData("input_data", true);
  data_storage->setData("input_data2", std::vector<tesseract::common::AnyPoly>{ false });
  auto future = server.run(pipeline, std::move(data_storage), false, "TaskflowExecutor");
  future->wait();

  const auto graph_node = std::dynamic_pointer_cast<const TaskComposerGraph>(
      static_cast<const TaskComposerGraph&>(pipeline).getNodeByName("TestConditionalGraphTask"));
  ASSERT_NE(graph_node, nullptr);
  const auto test_task = graph_node->getNodeByName("TestTask");
  ASSERT_NE(test_task, nullptr);
  EXPECT_EQ(test_task->getParentUUID(), graph_node->getUUID());
  const auto graph_storage_any = future->context->data_storage->getData(graph_node->getUUIDString());
  ASSERT_FALSE(graph_storage_any.isNull());
  const auto& graph_storage = graph_storage_any.as<TaskComposerDataStorage::Ptr>();
  EXPECT_TRUE(graph_storage->hasKey("output_data"));
  EXPECT_TRUE(graph_storage->hasKey("output_data2"));

  const auto abort_info = future->context->task_infos->getInfo(future->context->task_infos->getAbortingNode());
  const std::string abort_message = abort_info.has_value() ? abort_info->status_message : "No aborting node";
  SCOPED_TRACE(abort_message);
  EXPECT_EQ(future->context->isAborted(), false);
  EXPECT_EQ(future->context->isSuccessful(), true);
  EXPECT_EQ(future->context->task_infos->getInfoMap().size(), 7);
  EXPECT_TRUE(future->context->task_infos->getAbortingNode().is_nil());

  std::ofstream os1;
  os1.open(tesseract::common::getTempPath() + "task_composer_pipeline_with_conditional_child_graph_task.dot");
  EXPECT_NO_THROW(pipeline.dump(os1, nullptr, future->context->task_infos->getInfoMap()));  // NOLINT
  os1.close();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
