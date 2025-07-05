/**
 * @file task_composer_plugin_factory.cpp
 * @brief A plugin factory for producing a task composer
 *
 * @author Levi Armstrong
 * @date August 27, 2022
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
#include <utility>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/yaml_extensions.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <boost/algorithm/string.hpp>
#include <console_bridge/console.h>

static const std::string TESSERACT_TASK_COMPOSER_PLUGIN_DIRECTORIES_ENV = "TESSERACT_TASK_COMPOSER_PLUGIN_"
                                                                          "DIRECTORIES";
static const std::string TESSERACT_TASK_COMPOSER_PLUGINS_ENV = "TESSERACT_TASK_COMPOSER_PLUGINS";

namespace tesseract_planning
{
std::string TaskComposerExecutorFactory::getSection() { return "TaskExec"; }

std::string TaskComposerNodeFactory::getSection() { return "TaskNode"; }

struct TaskComposerPluginFactory::Implementation
{
  mutable std::map<std::string, TaskComposerExecutorFactory::Ptr> executor_factories;
  mutable std::map<std::string, TaskComposerNodeFactory::Ptr> node_factories;
  tesseract_common::PluginInfoContainer executor_plugin_info;
  tesseract_common::PluginInfoContainer task_plugin_info;
  boost_plugin_loader::PluginLoader plugin_loader;
};

TaskComposerPluginFactory::TaskComposerPluginFactory() : impl_(std::make_unique<Implementation>())
{
  impl_->plugin_loader.search_libraries_env = TESSERACT_TASK_COMPOSER_PLUGINS_ENV;
  impl_->plugin_loader.search_paths_env = TESSERACT_TASK_COMPOSER_PLUGIN_DIRECTORIES_ENV;
  impl_->plugin_loader.search_paths.insert(TESSERACT_TASK_COMPOSER_PLUGIN_PATH);
  if (!std::string(TESSERACT_TASK_COMPOSER_PLUGINS).empty())
    boost::split(impl_->plugin_loader.search_libraries,
                 TESSERACT_TASK_COMPOSER_PLUGINS,
                 boost::is_any_of(":"),
                 boost::token_compress_on);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const tesseract_common::TaskComposerPluginInfo& config)
  : TaskComposerPluginFactory()
{
  loadConfig(config);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const YAML::Node& config,
                                                     const tesseract_common::ResourceLocator& locator)
  : TaskComposerPluginFactory()
{
  loadConfig(config, locator);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const std::filesystem::path& config,
                                                     const tesseract_common::ResourceLocator& locator)
  : TaskComposerPluginFactory()
{
  loadConfig(config, locator);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const std::string& config,
                                                     const tesseract_common::ResourceLocator& locator)
  : TaskComposerPluginFactory()
{
  loadConfig(config, locator);
}

// This prevents it from being defined inline.
// If not the forward declare of PluginLoader cause compiler error.
TaskComposerPluginFactory::~TaskComposerPluginFactory() = default;
TaskComposerPluginFactory::TaskComposerPluginFactory(TaskComposerPluginFactory&&) noexcept = default;
TaskComposerPluginFactory& TaskComposerPluginFactory::operator=(TaskComposerPluginFactory&&) noexcept = default;

void TaskComposerPluginFactory::loadConfig(const tesseract_common::TaskComposerPluginInfo& config)
{
  impl_->plugin_loader.search_libraries.insert(config.search_libraries.begin(), config.search_libraries.end());
  impl_->plugin_loader.search_paths.insert(config.search_paths.begin(), config.search_paths.end());

  impl_->executor_plugin_info.plugins.insert(config.executor_plugin_infos.plugins.begin(),
                                             config.executor_plugin_infos.plugins.end());
  impl_->executor_plugin_info.default_plugin = config.executor_plugin_infos.default_plugin;

  impl_->task_plugin_info.plugins.insert(config.task_plugin_infos.plugins.begin(),
                                         config.task_plugin_infos.plugins.end());
  impl_->task_plugin_info.default_plugin = config.task_plugin_infos.default_plugin;
}

void TaskComposerPluginFactory::loadConfig(const YAML::Node& config)
{
  if (const YAML::Node& plugin_info = config[tesseract_common::TaskComposerPluginInfo::CONFIG_KEY])
  {
    auto tc_plugin_info = plugin_info.as<tesseract_common::TaskComposerPluginInfo>();
    impl_->plugin_loader.search_paths.insert(tc_plugin_info.search_paths.begin(), tc_plugin_info.search_paths.end());
    impl_->plugin_loader.search_libraries.insert(tc_plugin_info.search_libraries.begin(),
                                                 tc_plugin_info.search_libraries.end());
    impl_->executor_plugin_info = tc_plugin_info.executor_plugin_infos;
    impl_->task_plugin_info = tc_plugin_info.task_plugin_infos;
  }
}

void TaskComposerPluginFactory::loadConfig(const YAML::Node& config, const tesseract_common::ResourceLocator& locator)
{
  loadConfig(tesseract_common::processYamlIncludeDirective(config, locator));
}

void TaskComposerPluginFactory::loadConfig(const std::filesystem::path& config,
                                           const tesseract_common::ResourceLocator& locator)
{
  loadConfig(tesseract_common::loadYamlFile(config.string(), locator));
}

void TaskComposerPluginFactory::loadConfig(const std::string& config, const tesseract_common::ResourceLocator& locator)
{
  loadConfig(tesseract_common::loadYamlString(config, locator));
}

void TaskComposerPluginFactory::addSearchPath(const std::string& path)
{
  impl_->plugin_loader.search_paths.insert(path);
}

std::set<std::string> TaskComposerPluginFactory::getSearchPaths() const
{
  return std::as_const(*impl_).plugin_loader.search_paths;
}

void TaskComposerPluginFactory::clearSearchPaths() { impl_->plugin_loader.search_paths.clear(); }

void TaskComposerPluginFactory::addSearchLibrary(const std::string& library_name)
{
  impl_->plugin_loader.search_libraries.insert(library_name);
}

std::set<std::string> TaskComposerPluginFactory::getSearchLibraries() const
{
  return std::as_const(*impl_).plugin_loader.search_libraries;
}

void TaskComposerPluginFactory::clearSearchLibraries() { impl_->plugin_loader.search_libraries.clear(); }

void TaskComposerPluginFactory::addTaskComposerExecutorPlugin(const std::string& name,
                                                              tesseract_common::PluginInfo plugin_info)
{
  impl_->executor_plugin_info.plugins[name] = std::move(plugin_info);
}

bool TaskComposerPluginFactory::hasTaskComposerExecutorPlugins() const
{
  return !std::as_const(*impl_).executor_plugin_info.plugins.empty();
}

tesseract_common::PluginInfoMap TaskComposerPluginFactory::getTaskComposerExecutorPlugins() const
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
                                                          tesseract_common::PluginInfo plugin_info)
{
  impl_->task_plugin_info.plugins[name] = std::move(plugin_info);
}

bool TaskComposerPluginFactory::hasTaskComposerNodePlugins() const
{
  return !std::as_const(*impl_).task_plugin_info.plugins.empty();
}

tesseract_common::PluginInfoMap TaskComposerPluginFactory::getTaskComposerNodePlugins() const
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
    CONSOLE_BRIDGE_logWarn("TaskComposerPluginFactory, tried to get task composer executor '%s' that does not "
                           "exist!",
                           name.c_str());
    return nullptr;
  }

  return createTaskComposerExecutor(name, cm_it->second);
}

std::unique_ptr<TaskComposerExecutor>
TaskComposerPluginFactory::createTaskComposerExecutor(const std::string& name,
                                                      const tesseract_common::PluginInfo& plugin_info) const
{
  try
  {
    auto& executor_factories = impl_->executor_factories;
    auto it = executor_factories.find(plugin_info.class_name);
    if (it != executor_factories.end())
      return it->second->create(name, plugin_info.config);

    auto plugin = impl_->plugin_loader.createInstance<TaskComposerExecutorFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    executor_factories[plugin_info.class_name] = plugin;
    return plugin->create(name, plugin_info.config);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s', Details: %s", plugin_info.class_name.c_str(), e.what());
    return nullptr;
  }
}

std::unique_ptr<TaskComposerNode> TaskComposerPluginFactory::createTaskComposerNode(const std::string& name) const
{
  const auto& task_plugin_info = impl_->task_plugin_info;
  auto cm_it = task_plugin_info.plugins.find(name);
  if (cm_it == task_plugin_info.plugins.end())
  {
    CONSOLE_BRIDGE_logWarn("TaskComposerPluginFactory, tried to get task composer node '%s' that does not "
                           "exist!",
                           name.c_str());
    return nullptr;
  }

  return createTaskComposerNode(name, cm_it->second);
}

std::unique_ptr<TaskComposerNode>
TaskComposerPluginFactory::createTaskComposerNode(const std::string& name,
                                                  const tesseract_common::PluginInfo& plugin_info) const
{
  try
  {
    auto& node_factories = impl_->node_factories;
    auto it = node_factories.find(plugin_info.class_name);
    if (it != node_factories.end())
      return it->second->create(name, plugin_info.config, *this);

    auto plugin = impl_->plugin_loader.createInstance<TaskComposerNodeFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    node_factories[plugin_info.class_name] = plugin;
    return plugin->create(name, plugin_info.config, *this);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s', Details: %s", plugin_info.class_name.c_str(), e.what());
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
  tesseract_common::TaskComposerPluginInfo tc_plugins;
  tc_plugins.search_paths = impl_->plugin_loader.search_paths;
  tc_plugins.search_libraries = impl_->plugin_loader.search_libraries;
  tc_plugins.executor_plugin_infos = impl_->executor_plugin_info;
  tc_plugins.task_plugin_infos = impl_->task_plugin_info;

  YAML::Node config;
  config[tesseract_common::TaskComposerPluginInfo::CONFIG_KEY] = tc_plugins;

  return config;
}
}  // namespace tesseract_planning
