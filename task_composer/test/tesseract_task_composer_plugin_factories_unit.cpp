/**
 * @file teasseract_task_composer_plugin_factories_unit.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date Feb 17, 2023
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/task_composer/task_composer_executor.h>
#include <tesseract/task_composer/task_composer_node.h>
#include <tesseract/task_composer/task_composer_plugin_factory.h>
#include <tesseract/common/utils.h>
#include <tesseract/common/types.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registry.h>
#include <tesseract/common/yaml_utils.h>
#include <tesseract/common/yaml_extensions.h>

using namespace tesseract::task_composer;

namespace
{
const std::string TASK_COMPOSER_NODE_FACTORY_SCHEMA_KEY = "tesseract::task_composer::TaskComposerNodeFactory";
const std::string TASK_COMPOSER_EXECUTOR_FACTORY_SCHEMA_KEY = "tesseract::task_composer::TaskComposerExecutorFactory";

void expectRegisteredFactorySchemas(const std::vector<std::string>& factory_classes, const std::string& base_type)
{
  const auto registry = tesseract::common::SchemaRegistry::instance();
  ASSERT_FALSE(factory_classes.empty());

  for (const std::string& factory_class : factory_classes)
  {
    SCOPED_TRACE(factory_class);
    EXPECT_TRUE(registry->contains(factory_class));
    EXPECT_TRUE(registry->isDerivedFrom(base_type, factory_class));
    if (registry->contains(factory_class))
    {
      EXPECT_FALSE(registry->get(factory_class).empty());
    }
  }
}
}  // namespace

std::filesystem::path getTaskComposerConfigPath()
{
#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
  return std::filesystem::path(TESSERACT_TASK_COMPOSER_DIR) / "config/task_composer_plugins.yaml";
#else
  return std::filesystem::path(TESSERACT_TASK_COMPOSER_DIR) / "config/task_composer_plugins_no_trajopt_ifopt.yaml";
#endif
}

void runTaskComposerFactoryTest(TaskComposerPluginFactory& factory, YAML::Node plugin_config)
{
  const YAML::Node& plugin_info = plugin_config[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY];
  const YAML::Node& search_paths = plugin_info["search_paths"];
  const YAML::Node& search_libraries = plugin_info["search_libraries"];
  const YAML::Node& executor_plugins = plugin_info["executors"]["plugins"];
  const YAML::Node& task_plugins = plugin_info["tasks"]["plugins"];

  expectRegisteredFactorySchemas(factory.getAvailableTaskComposerNodePlugins(), TASK_COMPOSER_NODE_FACTORY_SCHEMA_KEY);
  expectRegisteredFactorySchemas(factory.getAvailableTaskComposerExecutorPlugins(),
                                 TASK_COMPOSER_EXECUTOR_FACTORY_SCHEMA_KEY);

  {
    std::vector<std::string> sp = factory.getSearchPaths();
    EXPECT_EQ(sp.size(), 2);

    for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
    {
      EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
    }
  }

  {
    std::vector<std::string> sl = factory.getSearchLibraries();
    EXPECT_EQ(sl.size(), 3);

    for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
    {
      EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
    }
  }

  EXPECT_EQ(executor_plugins.size(), 1);
  for (auto cm_it = executor_plugins.begin(); cm_it != executor_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();
    const auto factory_class = cm_it->second["class"].as<std::string>();
    EXPECT_TRUE(tesseract::common::SchemaRegistry::instance()->contains(factory_class));
    EXPECT_TRUE(tesseract::common::SchemaRegistry::instance()->isDerivedFrom(TASK_COMPOSER_EXECUTOR_FACTORY_SCHEMA_KEY,
                                                                             factory_class));

    TaskComposerExecutor::UPtr cm = factory.createTaskComposerExecutor(name);
    EXPECT_TRUE(cm != nullptr);
  }
#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
  EXPECT_EQ(task_plugins.size(), 36);
#else
  EXPECT_EQ(task_plugins.size(), 32);
#endif
  for (auto cm_it = task_plugins.begin(); cm_it != task_plugins.end(); ++cm_it)
  {
    auto name = cm_it->first.as<std::string>();
    const auto factory_class = cm_it->second["class"].as<std::string>();
    EXPECT_TRUE(tesseract::common::SchemaRegistry::instance()->contains(factory_class));
    EXPECT_TRUE(tesseract::common::SchemaRegistry::instance()->isDerivedFrom(TASK_COMPOSER_NODE_FACTORY_SCHEMA_KEY,
                                                                             factory_class));

    TaskComposerNode::UPtr cm = factory.createTaskComposerNode(name);
    EXPECT_TRUE(cm != nullptr);
  }

  factory.saveConfig(std::filesystem::path(tesseract::common::getTempPath()) / "task_composer_plugins_export.yaml");

  // Failures
  {
    TaskComposerExecutor::UPtr cm = factory.createTaskComposerExecutor("DoesNotExist");
    EXPECT_TRUE(cm == nullptr);
  }
  {
    TaskComposerNode::UPtr cm = factory.createTaskComposerNode("DoesNotExist");
    EXPECT_TRUE(cm == nullptr);
  }
  {
    tesseract::common::PluginInfo plugin_info;
    plugin_info.class_name = "DoesNotExistFactory";
    TaskComposerExecutor::UPtr cm = factory.createTaskComposerExecutor("DoesNotExist", plugin_info);
    EXPECT_TRUE(cm == nullptr);
  }
  {
    tesseract::common::PluginInfo plugin_info;
    plugin_info.class_name = "DoesNotExistFactory";
    TaskComposerNode::UPtr cm = factory.createTaskComposerNode("DoesNotExist", plugin_info);
    EXPECT_TRUE(cm == nullptr);
  }
}

TEST(TesseractTaskComposerFactoryUnit, LoadAndExportPluginTest)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  {  // File Path Construction
#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
    std::filesystem::path config_path(std::string(TESSERACT_TASK_COMPOSER_DIR) + "/config/"
                                                                                 "task_composer_plugins.yaml");
#else
    std::filesystem::path config_path(std::string(TESSERACT_TASK_COMPOSER_DIR) + "/config/"
                                                                                 "task_composer_plugins_no_"
                                                                                 "trajopt_"
                                                                                 "ifopt.yaml");
#endif
    TaskComposerPluginFactory factory(config_path, locator);
    YAML::Node plugin_config = tesseract::common::loadYamlFile(config_path.string(), locator);
    runTaskComposerFactoryTest(factory, plugin_config);

    auto export_config_path = std::filesystem::path(tesseract::common::getTempPath()) / "task_composer_plugins_"
                                                                                        "export.yaml";
    TaskComposerPluginFactory check_factory(export_config_path, locator);
    runTaskComposerFactoryTest(check_factory, plugin_config);
  }

  {  // String Constructor
#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
    std::filesystem::path config_path(std::string(TESSERACT_TASK_COMPOSER_DIR) + "/config/"
                                                                                 "task_composer_plugins.yaml");
#else
    std::filesystem::path config_path(std::string(TESSERACT_TASK_COMPOSER_DIR) + "/config/"
                                                                                 "task_composer_plugins_no_"
                                                                                 "trajopt_"
                                                                                 "ifopt.yaml");
#endif

    TaskComposerPluginFactory factory(tesseract::common::fileToString(config_path), locator);
    YAML::Node plugin_config = YAML::LoadFile(config_path.string());
    runTaskComposerFactoryTest(factory, plugin_config);

    auto export_config_path = std::filesystem::path(tesseract::common::getTempPath()) / "task_composer_plugins_"
                                                                                        "export.yaml";
    TaskComposerPluginFactory check_factory(export_config_path, locator);
    runTaskComposerFactoryTest(check_factory, plugin_config);
  }

  {  // YAML Node Constructor
#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
    std::filesystem::path config_path(std::string(TESSERACT_TASK_COMPOSER_DIR) + "/config/"
                                                                                 "task_composer_plugins.yaml");
#else
    std::filesystem::path config_path(std::string(TESSERACT_TASK_COMPOSER_DIR) + "/config/"
                                                                                 "task_composer_plugins_no_"
                                                                                 "trajopt_"
                                                                                 "ifopt.yaml");
#endif

    YAML::Node plugin_config = YAML::LoadFile(config_path.string());
    TaskComposerPluginFactory factory(plugin_config, locator);
    runTaskComposerFactoryTest(factory, plugin_config);

    auto export_config_path = std::filesystem::path(tesseract::common::getTempPath()) / "task_composer_plugins_"
                                                                                        "export.yaml";
    TaskComposerPluginFactory check_factory(export_config_path, locator);
    runTaskComposerFactoryTest(check_factory, plugin_config);
  }

  // TaskComposerPluginInfo Constructor
  {
#ifdef TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT
    std::filesystem::path config_path(std::string(TESSERACT_TASK_COMPOSER_DIR) + "/config/"
                                                                                 "task_composer_plugins.yaml");
#else
    std::filesystem::path config_path(std::string(TESSERACT_TASK_COMPOSER_DIR) + "/config/"
                                                                                 "task_composer_plugins_no_"
                                                                                 "trajopt_"
                                                                                 "ifopt.yaml");
#endif

    YAML::Node plugin_config = YAML::LoadFile(config_path.string());
    const YAML::Node& plugins = plugin_config[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY];
    const auto search_paths = plugins["search_paths"].as<std::vector<std::string>>();
    const auto search_libraries = plugins["search_libraries"].as<std::vector<std::string>>();

    tesseract::common::TaskComposerPluginInfo info;
    info.search_paths.insert(info.search_paths.end(), search_paths.begin(), search_paths.end());
    info.search_libraries.insert(info.search_libraries.end(), search_libraries.begin(), search_libraries.end());
    info.task_plugin_infos.plugins = plugins["tasks"]["plugins"].as<tesseract::common::PluginInfoMap>();
    info.executor_plugin_infos.plugins = plugins["executors"]["plugins"].as<tesseract::common::PluginInfoMap>();
    info.executor_plugin_infos.default_plugin = plugins["executors"]["default"].as<std::string>();

    TaskComposerPluginFactory factory(info);
    runTaskComposerFactoryTest(factory, plugin_config);

    auto export_config_path = std::filesystem::path(tesseract::common::getTempPath()) / "task_composer_plugins_"
                                                                                        "export.yaml";
    TaskComposerPluginFactory check_factory(export_config_path, locator);
    runTaskComposerFactoryTest(check_factory, plugin_config);
  }
}

TEST(TesseractTaskComposerFactoryUnit, MissingPluginsSectionThrows)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  YAML::Node config;
  config[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY]["executors"]["default"] = "executor";

  EXPECT_THROW(TaskComposerPluginFactory(config, locator), std::runtime_error);
}

TEST(TesseractTaskComposerFactoryUnit, InvalidBuiltInPluginConfigThrows)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  const std::filesystem::path config_path = getTaskComposerConfigPath();
  YAML::Node config = YAML::LoadFile(config_path.string());
  YAML::Node plugins = config[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY]["executors"]["plugins"];
  plugins["TaskflowExecutor"]["config"]["threads"] = 0;

  EXPECT_THROW(TaskComposerPluginFactory(config, locator), std::runtime_error);
}

TEST(TesseractTaskComposerFactoryUnit, MissingRequiredGraphInputThrows)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  YAML::Node config = YAML::LoadFile(getTaskComposerConfigPath().string());
  TaskComposerPluginFactory valid_factory(config, locator);
  YAML::Node inputs = config[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY]["tasks"]["plugins"]
                            ["DescartesFTask"]["config"]["inputs"];
  ASSERT_TRUE(inputs.remove("environment"));
  ASSERT_FALSE(inputs["environment"]);

  auto schema = tesseract::common::SchemaRegistry::instance()->get("PipelineTaskFactory");
  EXPECT_FALSE(
      schema
          .applyConfig(config[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY]["tasks"]["plugins"]["DescartesFTas"
                                                                                                         "k"]["config"])
          .empty());

  EXPECT_THROW(TaskComposerPluginFactory(config, locator), std::runtime_error);
}

TEST(TesseractTaskComposerFactoryUnit, ForEachTaskFactorySchema)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  TaskComposerPluginFactory factory(getTaskComposerConfigPath(), locator);
  const auto registry = tesseract::common::SchemaRegistry::instance();

  ASSERT_TRUE(registry->contains("ForEachTaskFactory"));
  EXPECT_TRUE(registry->isDerivedFrom(TASK_COMPOSER_NODE_FACTORY_SCHEMA_KEY, "ForEachTaskFactory"));

  auto schema = registry->get("ForEachTaskFactory");
  EXPECT_TRUE(schema
                  .applyConfig(YAML::Load(R"(
inputs: {container: input_data}
outputs: {container: output_data}
operation:
  input_port: keys
  output_port: keys
  class: RemapTaskFactory
  config:
    copy: true
    inputs: {storage_keys: [input]}
    outputs: {storage_keys: [output]}
)"))
                  .empty());

  auto invalid_schema = registry->get("ForEachTaskFactory");
  EXPECT_FALSE(invalid_schema
                   .applyConfig(YAML::Load(R"(
inputs: {container: input_data}
outputs: {container: output_data}
operation:
  input_port: program
  output_port: program
  class: DoesNotExistFactory
)"))
                   .empty());

  auto invalid_config_schema = registry->get("ForEachTaskFactory");
  EXPECT_FALSE(invalid_config_schema
                   .applyConfig(YAML::Load(R"(
inputs: {container: input_data}
outputs: {container: output_data}
operation:
  input_port: keys
  output_port: keys
  class: RemapTaskFactory
  config:
    inputs: {storage_keys: [input]}
    outputs: {storage_keys: [output]}
    unsupported: true
)"))
                   .empty());
}

TEST(TesseractTaskComposerFactoryUnit, FailedYamlReloadPreservesFactoryState)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  const std::filesystem::path config_path = getTaskComposerConfigPath();
  YAML::Node valid_config = tesseract::common::loadYamlFile(config_path.string(), locator);
  TaskComposerPluginFactory factory(valid_config, locator);

  const auto initial_search_paths = factory.getSearchPaths();
  const auto initial_search_libraries = factory.getSearchLibraries();
  const auto initial_executor_plugins = factory.getTaskComposerExecutorPlugins();
  const auto initial_node_plugins = factory.getTaskComposerNodePlugins();
  const auto initial_default_executor = factory.getDefaultTaskComposerExecutorPlugin();
  const auto initial_default_node = factory.getDefaultTaskComposerNodePlugin();

  ASSERT_NE(factory.createTaskComposerExecutor(initial_default_executor), nullptr);
  ASSERT_NE(factory.createTaskComposerNode(initial_default_node), nullptr);

  YAML::Node malformed_discovery;
  auto malformed_plugin_info = malformed_discovery[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY];
  malformed_plugin_info["search_paths"] = "/tmp/plugins";
  malformed_plugin_info["search_libraries"].push_back("library_that_does_not_exist");

  try
  {
    factory.loadConfig(malformed_discovery, locator);
    FAIL() << "Expected plugin discovery validation to fail";
  }
  catch (const std::runtime_error& error)
  {
    const std::string message = error.what();
    EXPECT_NE(message.find("Plugin discovery validation failed"), std::string::npos);
    EXPECT_NE(message.find("search_paths"), std::string::npos);
  }

  EXPECT_EQ(factory.getSearchPaths(), initial_search_paths);
  EXPECT_EQ(factory.getSearchLibraries(), initial_search_libraries);
  EXPECT_EQ(factory.getTaskComposerExecutorPlugins(), initial_executor_plugins);
  EXPECT_EQ(factory.getTaskComposerNodePlugins(), initial_node_plugins);
  EXPECT_EQ(factory.getDefaultTaskComposerExecutorPlugin(), initial_default_executor);
  EXPECT_EQ(factory.getDefaultTaskComposerNodePlugin(), initial_default_node);
  EXPECT_NE(factory.createTaskComposerExecutor(initial_default_executor), nullptr);
  EXPECT_NE(factory.createTaskComposerNode(initial_default_node), nullptr);

  YAML::Node invalid_full_config = YAML::Clone(valid_config);
  auto invalid_plugin_info = invalid_full_config[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY];
  invalid_plugin_info["search_paths"].push_back("/tmp/path_that_must_not_be_committed");
  invalid_plugin_info["executors"]["plugins"]["TaskflowExecutor"]["config"]["threads"] = 0;

  try
  {
    factory.loadConfig(invalid_full_config, locator);
    FAIL() << "Expected complete plugin configuration validation to fail";
  }
  catch (const std::runtime_error& error)
  {
    EXPECT_NE(std::string(error.what()).find("Configuration validation failed"), std::string::npos);
  }

  EXPECT_EQ(factory.getSearchPaths(), initial_search_paths);
  EXPECT_EQ(factory.getSearchLibraries(), initial_search_libraries);
  EXPECT_EQ(factory.getTaskComposerExecutorPlugins(), initial_executor_plugins);
  EXPECT_EQ(factory.getTaskComposerNodePlugins(), initial_node_plugins);
  EXPECT_EQ(factory.getDefaultTaskComposerExecutorPlugin(), initial_default_executor);
  EXPECT_EQ(factory.getDefaultTaskComposerNodePlugin(), initial_default_node);
  EXPECT_NE(factory.createTaskComposerExecutor(initial_default_executor), nullptr);
  EXPECT_NE(factory.createTaskComposerNode(initial_default_node), nullptr);

  YAML::Node invalid_task_config = YAML::Clone(valid_config);
  auto task_plugins = invalid_task_config[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY]["tasks"]["plugins"];
  ASSERT_TRUE(task_plugins.IsMap());
  ASSERT_FALSE(task_plugins.begin() == task_plugins.end());
  task_plugins.begin()->second["config"]["unsupported_option"] = true;

  try
  {
    factory.loadConfig(invalid_task_config, locator);
    FAIL() << "Expected task plugin configuration validation to fail";
  }
  catch (const std::runtime_error& error)
  {
    EXPECT_NE(std::string(error.what()).find("Configuration validation failed"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find("unsupported_option"), std::string::npos);
  }

  EXPECT_EQ(factory.getSearchPaths(), initial_search_paths);
  EXPECT_EQ(factory.getSearchLibraries(), initial_search_libraries);
  EXPECT_EQ(factory.getTaskComposerExecutorPlugins(), initial_executor_plugins);
  EXPECT_EQ(factory.getTaskComposerNodePlugins(), initial_node_plugins);
  EXPECT_EQ(factory.getDefaultTaskComposerExecutorPlugin(), initial_default_executor);
  EXPECT_EQ(factory.getDefaultTaskComposerNodePlugin(), initial_default_node);
  EXPECT_NE(factory.createTaskComposerExecutor(initial_default_executor), nullptr);
  EXPECT_NE(factory.createTaskComposerNode(initial_default_node), nullptr);

  EXPECT_NO_THROW(factory.loadConfig(valid_config, locator));
  EXPECT_NE(factory.createTaskComposerExecutor(initial_default_executor), nullptr);
  EXPECT_NE(factory.createTaskComposerNode(initial_default_node), nullptr);
}

TEST(TesseractTaskComposerFactoryUnit, PluginFactorAPIUnit)  // NOLINT
{
  TaskComposerPluginFactory factory;
  EXPECT_FALSE(factory.getSearchPaths().empty());
  EXPECT_EQ(factory.getSearchPaths().size(), 1);
  EXPECT_FALSE(factory.getSearchLibraries().empty());
  EXPECT_EQ(factory.getSearchLibraries().size(), 3);
  EXPECT_EQ(factory.getTaskComposerExecutorPlugins().size(), 0);
  EXPECT_EQ(factory.getTaskComposerNodePlugins().size(), 0);
  EXPECT_ANY_THROW(factory.getDefaultTaskComposerExecutorPlugin());  // NOLINT
  EXPECT_ANY_THROW(factory.getDefaultTaskComposerNodePlugin());      // NOLINT
  EXPECT_FALSE(factory.hasTaskComposerExecutorPlugins());
  EXPECT_FALSE(factory.hasTaskComposerNodePlugins());

  factory.addSearchPath("/usr/local/lib");
  EXPECT_EQ(factory.getSearchPaths().size(), 2);
  EXPECT_EQ(factory.getSearchLibraries().size(), 3);

  factory.addSearchLibrary("tesseract_collision");
  EXPECT_EQ(factory.getSearchPaths().size(), 2);
  EXPECT_EQ(factory.getSearchLibraries().size(), 4);

  {
    tesseract::common::PluginInfoMap map = factory.getTaskComposerExecutorPlugins();
    EXPECT_TRUE(map.find("NotFound") == map.end());

    tesseract::common::PluginInfo pi;
    pi.class_name = "TestTaskComposerExecutorFactory";
    factory.addTaskComposerExecutorPlugin("TestTaskComposerExecutor", pi);
    EXPECT_EQ(factory.getTaskComposerExecutorPlugins().size(), 1);
    EXPECT_TRUE(factory.hasTaskComposerExecutorPlugins());

    map = factory.getTaskComposerExecutorPlugins();
    EXPECT_TRUE(map.find("TestTaskComposerExecutor") != map.end());
    EXPECT_EQ(factory.getDefaultTaskComposerExecutorPlugin(), "TestTaskComposerExecutor");

    tesseract::common::PluginInfo pi2;
    pi2.class_name = "Test2TaskComposerExecutorFactory";
    factory.addTaskComposerExecutorPlugin("Test2TaskComposerExecutor", pi2);
    EXPECT_EQ(factory.getTaskComposerExecutorPlugins().size(), 2);
    EXPECT_TRUE(factory.hasTaskComposerExecutorPlugins());

    map = factory.getTaskComposerExecutorPlugins();
    EXPECT_TRUE(map.find("Test2TaskComposerExecutor") != map.end());
    EXPECT_EQ(factory.getDefaultTaskComposerExecutorPlugin(), "Test2TaskComposerExecutor");
    factory.setDefaultTaskComposerExecutorPlugin("Test2TaskComposerExecutor");
    EXPECT_EQ(factory.getDefaultTaskComposerExecutorPlugin(), "Test2TaskComposerExecutor");

    factory.removeTaskComposerExecutorPlugin("TestTaskComposerExecutor");
    map = factory.getTaskComposerExecutorPlugins();
    EXPECT_TRUE(map.find("Test2TaskComposerExecutor") != map.end());
    EXPECT_EQ(factory.getTaskComposerExecutorPlugins().size(), 1);
    // The default was removed so it should now be the first solver
    EXPECT_EQ(factory.getDefaultTaskComposerExecutorPlugin(), "Test2TaskComposerExecutor");

    // Failures
    EXPECT_ANY_THROW(factory.removeTaskComposerExecutorPlugin("DoesNotExist"));      // NOLINT
    EXPECT_ANY_THROW(factory.setDefaultTaskComposerExecutorPlugin("DoesNotExist"));  // NOLINT
  }

  {
    tesseract::common::PluginInfoMap map = factory.getTaskComposerNodePlugins();
    EXPECT_TRUE(map.find("NotFound") == map.end());

    tesseract::common::PluginInfo pi;
    pi.class_name = "TestTaskComposerNodeFactory";
    factory.addTaskComposerNodePlugin("TestTaskComposerNode", pi);
    EXPECT_EQ(factory.getTaskComposerNodePlugins().size(), 1);
    EXPECT_TRUE(factory.hasTaskComposerNodePlugins());

    map = factory.getTaskComposerNodePlugins();
    EXPECT_TRUE(map.find("TestTaskComposerNode") != map.end());
    EXPECT_EQ(factory.getDefaultTaskComposerNodePlugin(), "TestTaskComposerNode");

    tesseract::common::PluginInfo pi2;
    pi2.class_name = "Test2TaskComposerNodeFactory";
    factory.addTaskComposerNodePlugin("Test2TaskComposerNode", pi2);
    EXPECT_EQ(factory.getTaskComposerNodePlugins().size(), 2);
    EXPECT_TRUE(factory.hasTaskComposerNodePlugins());

    map = factory.getTaskComposerNodePlugins();
    EXPECT_TRUE(map.find("Test2TaskComposerNode") != map.end());
    EXPECT_EQ(factory.getDefaultTaskComposerNodePlugin(), "Test2TaskComposerNode");
    factory.setDefaultTaskComposerNodePlugin("Test2TaskComposerNode");
    EXPECT_EQ(factory.getDefaultTaskComposerNodePlugin(), "Test2TaskComposerNode");

    factory.removeTaskComposerNodePlugin("TestTaskComposerNode");

    map = factory.getTaskComposerNodePlugins();
    EXPECT_TRUE(map.find("Test2TaskComposerNode") != map.end());
    EXPECT_EQ(factory.getTaskComposerNodePlugins().size(), 1);
    // The default was removed so it should now be the first solver
    EXPECT_EQ(factory.getDefaultTaskComposerNodePlugin(), "Test2TaskComposerNode");

    // Failures
    EXPECT_ANY_THROW(factory.removeTaskComposerNodePlugin("DoesNotExist"));      // NOLINT
    EXPECT_ANY_THROW(factory.setDefaultTaskComposerNodePlugin("DoesNotExist"));  // NOLINT
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
