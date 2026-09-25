/**
 * @file task_composer_plugin_factory.cpp
 * @brief A plugin factory for producing a task composer
 *
 * @author Levi Armstrong
 * @date August 27, 2022
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#include <yaml-cpp/yaml.h>
#include <utility>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/yaml_utils.h>
#include <tesseract/common/yaml_extensions.h>
#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registry.h>
#include <tesseract/task_composer/task_composer_plugin_factory.h>
#include <tesseract/task_composer/task_composer_node.h>
#include <tesseract/task_composer/task_composer_executor.h>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <boost/algorithm/string.hpp>
#include <tesseract/common/logging.h>

static const std::string TESSERACT_TASK_COMPOSER_PLUGIN_DIRECTORIES_ENV = "TESSERACT_TASK_COMPOSER_PLUGIN_"
                                                                          "DIRECTORIES";
static const std::string TESSERACT_TASK_COMPOSER_PLUGINS_ENV = "TESSERACT_TASK_COMPOSER_PLUGINS";

namespace tesseract::task_composer
{
std::string TaskComposerExecutorFactory::getSection() { return "TaskExec"; }

tesseract::common::PropertyTree TaskComposerExecutorFactory::schema() const
{
  return tesseract::common::PropertyTreeBuilder().build();
}

std::unique_ptr<TaskComposerExecutor> TaskComposerExecutorFactory::create(const std::string& name,
                                                                          const YAML::Node& config) const
{
  auto validated_config = schema();
  auto errors = validated_config.applyConfig(config);
  if (!errors.empty())
    throw tesseract::common::PropertyTreeValidationError(std::move(errors));

  return createImpl(name, validated_config);
}

std::string TaskComposerNodeFactory::getSection() { return "TaskNode"; }

tesseract::common::PropertyTree TaskComposerNodeFactory::schema() const
{
  return tesseract::common::PropertyTreeBuilder().build();
}

std::unique_ptr<TaskComposerNode> TaskComposerNodeFactory::create(const std::string& name,
                                                                  const YAML::Node& config,
                                                                  const TaskComposerPluginFactory& plugin_factory) const
{
  auto validated_config = schema();
  auto errors = validated_config.applyConfig(config);
  if (!errors.empty())
    throw tesseract::common::PropertyTreeValidationError(std::move(errors));

  return createImpl(name, validated_config, plugin_factory);
}

struct TaskComposerPluginFactory::Implementation
{
  mutable std::map<std::string, TaskComposerExecutorFactory::Ptr> executor_factories;
  mutable std::map<std::string, TaskComposerNodeFactory::Ptr> node_factories;
  tesseract::common::PluginInfoContainer executor_plugin_info;
  tesseract::common::PluginInfoContainer task_plugin_info;
  boost_plugin_loader::PluginLoader plugin_loader;
};

TaskComposerPluginFactory::TaskComposerPluginFactory() : impl_(std::make_unique<Implementation>())
{
  impl_->plugin_loader.search_libraries_env = TESSERACT_TASK_COMPOSER_PLUGINS_ENV;
  impl_->plugin_loader.search_paths_env = TESSERACT_TASK_COMPOSER_PLUGIN_DIRECTORIES_ENV;
  impl_->plugin_loader.search_paths.emplace_back(TESSERACT_TASK_COMPOSER_PLUGIN_PATH);
  if (!std::string(TESSERACT_TASK_COMPOSER_PLUGINS).empty())
    boost::split(impl_->plugin_loader.search_libraries,
                 TESSERACT_TASK_COMPOSER_PLUGINS,
                 boost::is_any_of(":"),
                 boost::token_compress_on);

  tesseract::common::removeDuplicates(impl_->plugin_loader.search_paths);
  tesseract::common::removeDuplicates(impl_->plugin_loader.search_libraries);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const tesseract::common::TaskComposerPluginInfo& config)
  : TaskComposerPluginFactory()
{
  loadConfig(config);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const YAML::Node& config,
                                                     const tesseract::common::ResourceLocator& locator)
  : TaskComposerPluginFactory()
{
  loadConfig(config, locator);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const std::filesystem::path& config,
                                                     const tesseract::common::ResourceLocator& locator)
  : TaskComposerPluginFactory()
{
  loadConfig(config, locator);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const std::string& config,
                                                     const tesseract::common::ResourceLocator& locator)
  : TaskComposerPluginFactory()
{
  loadConfig(config, locator);
}

// This prevents it from being defined inline.
// If not the forward declare of PluginLoader cause compiler error.
TaskComposerPluginFactory::~TaskComposerPluginFactory() = default;
TaskComposerPluginFactory::TaskComposerPluginFactory(TaskComposerPluginFactory&&) noexcept = default;
TaskComposerPluginFactory& TaskComposerPluginFactory::operator=(TaskComposerPluginFactory&&) noexcept = default;

void TaskComposerPluginFactory::loadConfig(const tesseract::common::TaskComposerPluginInfo& config)
{
  impl_->plugin_loader.search_libraries.insert(
      impl_->plugin_loader.search_libraries.end(), config.search_libraries.begin(), config.search_libraries.end());
  impl_->plugin_loader.search_paths.insert(
      impl_->plugin_loader.search_paths.end(), config.search_paths.begin(), config.search_paths.end());

  impl_->executor_plugin_info.plugins.insert(config.executor_plugin_infos.plugins.begin(),
                                             config.executor_plugin_infos.plugins.end());
  impl_->executor_plugin_info.default_plugin = config.executor_plugin_infos.default_plugin;

  impl_->task_plugin_info.plugins.insert(config.task_plugin_infos.plugins.begin(),
                                         config.task_plugin_infos.plugins.end());
  impl_->task_plugin_info.default_plugin = config.task_plugin_infos.default_plugin;

  tesseract::common::removeDuplicates(impl_->plugin_loader.search_paths);
  tesseract::common::removeDuplicates(impl_->plugin_loader.search_libraries);
}

void TaskComposerPluginFactory::loadConfig(YAML::Node config)
{
  if (const YAML::Node& plugin_info = config[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY])
  {
    YAML::Node plugin_info_for_decode = YAML::Clone(plugin_info);

    // Stage 1 validates only the metadata required to discover plugin schemas.
    auto discovery_schema = YAML::convert<tesseract::common::PluginDiscoveryInfo>::schema();
    YAML::Node plugin_info_for_discovery_validation = YAML::Clone(plugin_info_for_decode);
    auto discovery_errors = discovery_schema.applyConfig(plugin_info_for_discovery_validation, true);
    if (!discovery_errors.empty())
    {
      std::string error_msg = "TaskComposerPluginFactory: Plugin discovery validation failed:\n";
      for (const auto& error : discovery_errors)
        error_msg += "  - " + error + "\n";

      throw std::runtime_error(error_msg);
    }

    const auto discovery_info = plugin_info_for_decode.as<tesseract::common::PluginDiscoveryInfo>();
    boost_plugin_loader::PluginLoader candidate_loader = impl_->plugin_loader;
    candidate_loader.search_paths.insert(
        candidate_loader.search_paths.end(), discovery_info.search_paths.begin(), discovery_info.search_paths.end());
    candidate_loader.search_libraries.insert(candidate_loader.search_libraries.end(),
                                             discovery_info.search_libraries.begin(),
                                             discovery_info.search_libraries.end());
    tesseract::common::removeDuplicates(candidate_loader.search_paths);
    tesseract::common::removeDuplicates(candidate_loader.search_libraries);

    // Loading the libraries runs their static schema registrations before strict validation.
    // The registry retains their lifetime handles alongside the registered schemas.
    tesseract::common::SchemaRegistry::instance()->loadAndRetainPluginLibraries(candidate_loader);

    // Stage 2 strictly validates the complete configuration after plugin schemas are registered.
    auto schema = YAML::convert<tesseract::common::TaskComposerPluginInfo>::schema();
    auto config_tree = schema;
    YAML::Node plugin_info_for_validation = YAML::Clone(plugin_info_for_decode);
    auto errors = config_tree.applyConfig(plugin_info_for_validation, false);
    if (!errors.empty())
    {
      std::string error_msg = "TaskComposerPluginFactory: Configuration validation failed:\n";
      for (const auto& error : errors)
        error_msg += "  - " + error + "\n";

      throw std::runtime_error(error_msg);
    }

    auto tc_plugin_info = plugin_info_for_decode.as<tesseract::common::TaskComposerPluginInfo>();
    impl_->executor_plugin_info = std::move(tc_plugin_info.executor_plugin_infos);
    impl_->task_plugin_info = std::move(tc_plugin_info.task_plugin_infos);
    impl_->plugin_loader = std::move(candidate_loader);
  }
}

void TaskComposerPluginFactory::loadConfig(YAML::Node config, const tesseract::common::ResourceLocator& locator)
{
  tesseract::common::processYamlIncludeDirective(config, locator);
  loadConfig(config);
}

void TaskComposerPluginFactory::loadConfig(const std::filesystem::path& config,
                                           const tesseract::common::ResourceLocator& locator)
{
  loadConfig(tesseract::common::loadYamlFile(config.string(), locator));
}

void TaskComposerPluginFactory::loadConfig(const std::string& config, const tesseract::common::ResourceLocator& locator)
{
  loadConfig(tesseract::common::loadYamlString(config, locator));
}

void TaskComposerPluginFactory::addSearchPath(const std::string& path)
{
  auto& v = impl_->plugin_loader.search_paths;
  if (std::find(v.begin(), v.end(), path) == v.end())
    v.push_back(path);
}

std::vector<std::string> TaskComposerPluginFactory::getSearchPaths() const
{
  return std::as_const(*impl_).plugin_loader.search_paths;
}

void TaskComposerPluginFactory::clearSearchPaths() { impl_->plugin_loader.search_paths.clear(); }

void TaskComposerPluginFactory::addSearchLibrary(const std::string& library_name)
{
  auto& v = impl_->plugin_loader.search_libraries;
  if (std::find(v.begin(), v.end(), library_name) == v.end())
    v.push_back(library_name);
}

std::vector<std::string> TaskComposerPluginFactory::getSearchLibraries() const
{
  return std::as_const(*impl_).plugin_loader.search_libraries;
}

void TaskComposerPluginFactory::clearSearchLibraries() { impl_->plugin_loader.search_libraries.clear(); }

void TaskComposerPluginFactory::addTaskComposerExecutorPlugin(const std::string& name,
                                                              tesseract::common::PluginInfo plugin_info)
{
  impl_->executor_plugin_info.plugins[name] = std::move(plugin_info);
}

bool TaskComposerPluginFactory::hasTaskComposerExecutorPlugins() const
{
  return !std::as_const(*impl_).executor_plugin_info.plugins.empty();
}

tesseract::common::PluginInfoMap TaskComposerPluginFactory::getTaskComposerExecutorPlugins() const
{
  return std::as_const(*impl_).executor_plugin_info.plugins;
}

void TaskComposerPluginFactory::removeTaskComposerExecutorPlugin(const std::string& name)
{
  auto& executor_plugin_info = impl_->executor_plugin_info;
  auto cm_it = executor_plugin_info.plugins.find(name);
  if (cm_it == executor_plugin_info.plugins.end())
    throw std::runtime_error("TaskComposerPluginFactory, tried to remove task composer executor '" + name +
                             "' that does not exist!");

  executor_plugin_info.plugins.erase(cm_it);

  if (executor_plugin_info.default_plugin == name)
    executor_plugin_info.default_plugin.clear();
}

void TaskComposerPluginFactory::setDefaultTaskComposerExecutorPlugin(const std::string& name)
{
  auto& executor_plugin_info = impl_->executor_plugin_info;
  auto cm_it = executor_plugin_info.plugins.find(name);
  if (cm_it == executor_plugin_info.plugins.end())
    throw std::runtime_error("TaskComposerPluginFactory, tried to set default task composer executor '" + name +
                             "' that does not exist!");

  executor_plugin_info.default_plugin = name;
}

std::string TaskComposerPluginFactory::getDefaultTaskComposerExecutorPlugin() const
{
  const auto& executor_plugin_info = impl_->executor_plugin_info;
  if (executor_plugin_info.plugins.empty())
    throw std::runtime_error("TaskComposerPluginFactory, tried to get default task composer executor but none "
                             "exist!");

  if (executor_plugin_info.default_plugin.empty())
    return executor_plugin_info.plugins.begin()->first;

  return executor_plugin_info.default_plugin;
}

void TaskComposerPluginFactory::addTaskComposerNodePlugin(const std::string& name,
                                                          tesseract::common::PluginInfo plugin_info)
{
  impl_->task_plugin_info.plugins[name] = std::move(plugin_info);
}

bool TaskComposerPluginFactory::hasTaskComposerNodePlugins() const
{
  return !std::as_const(*impl_).task_plugin_info.plugins.empty();
}

tesseract::common::PluginInfoMap TaskComposerPluginFactory::getTaskComposerNodePlugins() const
{
  return std::as_const(*impl_).task_plugin_info.plugins;
}

void TaskComposerPluginFactory::removeTaskComposerNodePlugin(const std::string& name)
{
  auto& task_plugin_info = impl_->task_plugin_info;
  auto cm_it = task_plugin_info.plugins.find(name);
  if (cm_it == task_plugin_info.plugins.end())
    throw std::runtime_error("TaskComposerPluginFactory, tried to remove task composer node '" + name +
                             "' that does not exist!");

  task_plugin_info.plugins.erase(cm_it);

  if (task_plugin_info.default_plugin == name)
    task_plugin_info.default_plugin.clear();
}

void TaskComposerPluginFactory::setDefaultTaskComposerNodePlugin(const std::string& name)
{
  auto& task_plugin_info = impl_->task_plugin_info;
  auto cm_it = task_plugin_info.plugins.find(name);
  if (cm_it == task_plugin_info.plugins.end())
    throw std::runtime_error("TaskComposerPluginFactory, tried to set default task composer node '" + name +
                             "' that does not exist!");

  task_plugin_info.default_plugin = name;
}

std::string TaskComposerPluginFactory::getDefaultTaskComposerNodePlugin() const
{
  const auto& task_plugin_info = impl_->task_plugin_info;
  if (task_plugin_info.plugins.empty())
    throw std::runtime_error("TaskComposerPluginFactory, tried to get default task composer node but none "
                             "exist!");

  if (task_plugin_info.default_plugin.empty())
    return task_plugin_info.plugins.begin()->first;

  return task_plugin_info.default_plugin;
}

std::unique_ptr<TaskComposerExecutor>
TaskComposerPluginFactory::createTaskComposerExecutor(const std::string& name) const
{
  const auto& executor_plugin_info = impl_->executor_plugin_info;
  auto cm_it = executor_plugin_info.plugins.find(name);
  if (cm_it == executor_plugin_info.plugins.end())
  {
    TESSERACT_LOG_WARN("TaskComposerPluginFactory, tried to get task composer executor '{}' that does not "
                       "exist!",
                       name.c_str());
    return nullptr;
  }

  return createTaskComposerExecutor(name, cm_it->second);
}

std::unique_ptr<TaskComposerExecutor>
TaskComposerPluginFactory::createTaskComposerExecutor(const std::string& name,
                                                      const tesseract::common::PluginInfo& plugin_info) const
{
  try
  {
    auto& executor_factories = impl_->executor_factories;
    auto it = executor_factories.find(plugin_info.class_name);
    if (it != executor_factories.end())
      return it->second->create(name, plugin_info.config);

    // Loading a factory may register schemas containing callbacks implemented by
    // its library. Retain the library until the schema registry is destroyed.
    tesseract::common::SchemaRegistry::instance()->loadAndRetainPluginLibraries(impl_->plugin_loader);
    auto plugin = impl_->plugin_loader.createInstance<TaskComposerExecutorFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      TESSERACT_LOG_WARN("Failed to load symbol '{}'", plugin_info.class_name.c_str());
      return nullptr;
    }
    executor_factories[plugin_info.class_name] = plugin;
    return plugin->create(name, plugin_info.config);
  }
  catch (const std::exception& e)
  {
    TESSERACT_LOG_WARN("Failed to load symbol '{}', Details: {}", plugin_info.class_name.c_str(), e.what());
    return nullptr;
  }
}

std::unique_ptr<TaskComposerNode> TaskComposerPluginFactory::createTaskComposerNode(const std::string& name) const
{
  const auto& task_plugin_info = impl_->task_plugin_info;
  auto cm_it = task_plugin_info.plugins.find(name);
  if (cm_it == task_plugin_info.plugins.end())
  {
    TESSERACT_LOG_WARN("TaskComposerPluginFactory, tried to get task composer node '{}' that does not "
                       "exist!",
                       name.c_str());
    return nullptr;
  }

  return createTaskComposerNode(name, cm_it->second);
}

std::unique_ptr<TaskComposerNode>
TaskComposerPluginFactory::createTaskComposerNode(const std::string& name,
                                                  const tesseract::common::PluginInfo& plugin_info) const
{
  try
  {
    auto& node_factories = impl_->node_factories;
    auto it = node_factories.find(plugin_info.class_name);
    if (it != node_factories.end())
      return it->second->create(name, plugin_info.config, *this);

    // Loading a factory may register schemas containing callbacks implemented by
    // its library. Retain the library until the schema registry is destroyed.
    tesseract::common::SchemaRegistry::instance()->loadAndRetainPluginLibraries(impl_->plugin_loader);
    auto plugin = impl_->plugin_loader.createInstance<TaskComposerNodeFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      TESSERACT_LOG_WARN("Failed to load symbol '{}'", plugin_info.class_name.c_str());
      return nullptr;
    }
    node_factories[plugin_info.class_name] = plugin;
    return plugin->create(name, plugin_info.config, *this);
  }
  catch (const std::exception& e)
  {
    TESSERACT_LOG_WARN("Failed to load symbol '{}', Details: {}", plugin_info.class_name.c_str(), e.what());
    return nullptr;
  }
}

void TaskComposerPluginFactory::saveConfig(const std::filesystem::path& file_path) const
{
  YAML::Node config = getConfig();
  std::ofstream fout(file_path.string());
  fout << config;
}

YAML::Node TaskComposerPluginFactory::getConfig() const
{
  tesseract::common::TaskComposerPluginInfo tc_plugins;
  tc_plugins.search_paths = impl_->plugin_loader.search_paths;
  tc_plugins.search_libraries = impl_->plugin_loader.search_libraries;
  tc_plugins.executor_plugin_infos = impl_->executor_plugin_info;
  tc_plugins.task_plugin_infos = impl_->task_plugin_info;

  YAML::Node config;
  config[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY] = tc_plugins;

  return config;
}

std::vector<std::string> TaskComposerPluginFactory::getAvailableTaskComposerNodePlugins() const
{
  // Plugin discovery loads libraries and runs their static schema registrations.
  tesseract::common::SchemaRegistry::instance()->loadAndRetainPluginLibraries(impl_->plugin_loader);
  return impl_->plugin_loader.getAvailablePlugins(TaskComposerNodeFactory::getSection());
}

std::vector<std::string> TaskComposerPluginFactory::getAvailableTaskComposerExecutorPlugins() const
{
  // Plugin discovery loads libraries and runs their static schema registrations.
  tesseract::common::SchemaRegistry::instance()->loadAndRetainPluginLibraries(impl_->plugin_loader);
  return impl_->plugin_loader.getAvailablePlugins(TaskComposerExecutorFactory::getSection());
}

}  // namespace tesseract::task_composer
