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

#include <tesseract_common/plugin_loader.hpp>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

static const std::string TESSERACT_TASK_COMPOSER_PLUGIN_DIRECTORIES_ENV = "TESSERACT_TASK_COMPOSER_PLUGIN_"
                                                                          "DIRECTORIES";
static const std::string TESSERACT_TASK_COMPOSER_PLUGINS_ENV = "TESSERACT_TASK_COMPOSER_PLUGINS";

namespace tesseract_planning
{
const std::string TaskComposerExecutorFactory::SECTION_NAME = "TaskExec";
const std::string TaskComposerNodeFactory::SECTION_NAME = "TaskNode";

TaskComposerPluginFactory::TaskComposerPluginFactory()
{
  plugin_loader_.search_libraries_env = TESSERACT_TASK_COMPOSER_PLUGINS_ENV;
  plugin_loader_.search_paths_env = TESSERACT_TASK_COMPOSER_PLUGIN_DIRECTORIES_ENV;
  plugin_loader_.search_paths.insert(TESSERACT_TASK_COMPOSER_PLUGIN_PATH);
  if (!std::string(TESSERACT_TASK_COMPOSER_PLUGINS).empty())
    boost::split(plugin_loader_.search_libraries,
                 TESSERACT_TASK_COMPOSER_PLUGINS,
                 boost::is_any_of(":"),
                 boost::token_compress_on);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const tesseract_common::TaskComposerPluginInfo& config)
  : TaskComposerPluginFactory()
{
  loadConfig(config);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const YAML::Node& config) : TaskComposerPluginFactory()
{
  loadConfig(config);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const tesseract_common::fs::path& config)
  : TaskComposerPluginFactory()
{
  loadConfig(config);
}

TaskComposerPluginFactory::TaskComposerPluginFactory(const std::string& config) : TaskComposerPluginFactory()
{
  loadConfig(config);
}

// This prevents it from being defined inline.
// If not the forward declare of PluginLoader cause compiler error.
TaskComposerPluginFactory::~TaskComposerPluginFactory() = default;

void TaskComposerPluginFactory::loadConfig(const tesseract_common::TaskComposerPluginInfo& config)
{
  plugin_loader_.search_libraries.insert(config.search_libraries.begin(), config.search_libraries.end());
  plugin_loader_.search_paths.insert(config.search_paths.begin(), config.search_paths.end());

  executor_plugin_info_.plugins.insert(config.executor_plugin_infos.plugins.begin(),
                                       config.executor_plugin_infos.plugins.end());
  executor_plugin_info_.default_plugin = config.executor_plugin_infos.default_plugin;

  task_plugin_info_.plugins.insert(config.task_plugin_infos.plugins.begin(), config.task_plugin_infos.plugins.end());
  task_plugin_info_.default_plugin = config.task_plugin_infos.default_plugin;
}

void TaskComposerPluginFactory::loadConfig(const YAML::Node& config)
{
  if (const YAML::Node& plugin_info = config[tesseract_common::TaskComposerPluginInfo::CONFIG_KEY])
  {
    auto tc_plugin_info = plugin_info.as<tesseract_common::TaskComposerPluginInfo>();
    plugin_loader_.search_paths.insert(tc_plugin_info.search_paths.begin(), tc_plugin_info.search_paths.end());
    plugin_loader_.search_libraries.insert(tc_plugin_info.search_libraries.begin(),
                                           tc_plugin_info.search_libraries.end());
    executor_plugin_info_ = tc_plugin_info.executor_plugin_infos;
    task_plugin_info_ = tc_plugin_info.task_plugin_infos;
  }
}

void TaskComposerPluginFactory::loadConfig(const tesseract_common::fs::path& config)
{
  loadConfig(YAML::LoadFile(config.string()));
}

void TaskComposerPluginFactory::loadConfig(const std::string& config) { loadConfig(YAML::Load(config)); }

void TaskComposerPluginFactory::addSearchPath(const std::string& path) { plugin_loader_.search_paths.insert(path); }

std::set<std::string> TaskComposerPluginFactory::getSearchPaths() const { return plugin_loader_.search_paths; }

void TaskComposerPluginFactory::clearSearchPaths() { plugin_loader_.search_paths.clear(); }

void TaskComposerPluginFactory::addSearchLibrary(const std::string& library_name)
{
  plugin_loader_.search_libraries.insert(library_name);
}

std::set<std::string> TaskComposerPluginFactory::getSearchLibraries() const { return plugin_loader_.search_libraries; }

void TaskComposerPluginFactory::clearSearchLibraries() { plugin_loader_.search_libraries.clear(); }

void TaskComposerPluginFactory::addTaskComposerExecutorPlugin(const std::string& name,
                                                              tesseract_common::PluginInfo plugin_info)
{
  executor_plugin_info_.plugins[name] = std::move(plugin_info);
}

bool TaskComposerPluginFactory::hasTaskComposerExecutorPlugins() const
{
  return !executor_plugin_info_.plugins.empty();
}

tesseract_common::PluginInfoMap TaskComposerPluginFactory::getTaskComposerExecutorPlugins() const
{
  return executor_plugin_info_.plugins;
}

void TaskComposerPluginFactory::removeTaskComposerExecutorPlugin(const std::string& name)
{
  auto cm_it = executor_plugin_info_.plugins.find(name);
  if (cm_it == executor_plugin_info_.plugins.end())
    throw std::runtime_error("TaskComposerPluginFactory, tried to remove task composer executor '" + name +
                             "' that does not exist!");

  executor_plugin_info_.plugins.erase(cm_it);

  if (executor_plugin_info_.default_plugin == name)
    executor_plugin_info_.default_plugin.clear();
}

void TaskComposerPluginFactory::setDefaultTaskComposerExecutorPlugin(const std::string& name)
{
  auto cm_it = executor_plugin_info_.plugins.find(name);
  if (cm_it == executor_plugin_info_.plugins.end())
    throw std::runtime_error("TaskComposerPluginFactory, tried to set default task composer executor '" + name +
                             "' that does not exist!");

  executor_plugin_info_.default_plugin = name;
}

std::string TaskComposerPluginFactory::getDefaultTaskComposerExecutorPlugin() const
{
  if (executor_plugin_info_.plugins.empty())
    throw std::runtime_error("TaskComposerPluginFactory, tried to get default task composer executor but none "
                             "exist!");

  if (executor_plugin_info_.default_plugin.empty())
    return executor_plugin_info_.plugins.begin()->first;

  return executor_plugin_info_.default_plugin;
}

void TaskComposerPluginFactory::addTaskComposerNodePlugin(const std::string& name,
                                                          tesseract_common::PluginInfo plugin_info)
{
  task_plugin_info_.plugins[name] = std::move(plugin_info);
}

bool TaskComposerPluginFactory::hasTaskComposerNodePlugins() const { return !task_plugin_info_.plugins.empty(); }

tesseract_common::PluginInfoMap TaskComposerPluginFactory::getTaskComposerNodePlugins() const
{
  return task_plugin_info_.plugins;
}

void TaskComposerPluginFactory::removeTaskComposerNodePlugin(const std::string& name)
{
  auto cm_it = task_plugin_info_.plugins.find(name);
  if (cm_it == task_plugin_info_.plugins.end())
    throw std::runtime_error("TaskComposerPluginFactory, tried to remove task composer node '" + name +
                             "' that does not exist!");

  task_plugin_info_.plugins.erase(cm_it);

  if (task_plugin_info_.default_plugin == name)
    task_plugin_info_.default_plugin.clear();
}

void TaskComposerPluginFactory::setDefaultTaskComposerNodePlugin(const std::string& name)
{
  auto cm_it = task_plugin_info_.plugins.find(name);
  if (cm_it == task_plugin_info_.plugins.end())
    throw std::runtime_error("TaskComposerPluginFactory, tried to set default task composer node '" + name +
                             "' that does not exist!");

  task_plugin_info_.default_plugin = name;
}

std::string TaskComposerPluginFactory::getDefaultTaskComposerNodePlugin() const
{
  if (task_plugin_info_.plugins.empty())
    throw std::runtime_error("TaskComposerPluginFactory, tried to get default task composer node but none "
                             "exist!");

  if (task_plugin_info_.default_plugin.empty())
    return task_plugin_info_.plugins.begin()->first;

  return task_plugin_info_.default_plugin;
}

TaskComposerExecutor::UPtr TaskComposerPluginFactory::createTaskComposerExecutor(const std::string& name) const
{
  auto cm_it = executor_plugin_info_.plugins.find(name);
  if (cm_it == executor_plugin_info_.plugins.end())
  {
    CONSOLE_BRIDGE_logWarn("TaskComposerPluginFactory, tried to get task composer executor '%s' that does not "
                           "exist!",
                           name.c_str());
    return nullptr;
  }

  return createTaskComposerExecutor(name, cm_it->second);
}

TaskComposerExecutor::UPtr
TaskComposerPluginFactory::createTaskComposerExecutor(const std::string& name,
                                                      const tesseract_common::PluginInfo& plugin_info) const
{
  try
  {
    auto it = executor_factories_.find(plugin_info.class_name);
    if (it != executor_factories_.end())
      return it->second->create(name, plugin_info.config);

    auto plugin = plugin_loader_.instantiate<TaskComposerExecutorFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    executor_factories_[plugin_info.class_name] = plugin;
    return plugin->create(name, plugin_info.config);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s', Details: %s", plugin_info.class_name.c_str(), e.what());
    return nullptr;
  }
}

TaskComposerNode::UPtr TaskComposerPluginFactory::createTaskComposerNode(const std::string& name) const
{
  auto cm_it = task_plugin_info_.plugins.find(name);
  if (cm_it == task_plugin_info_.plugins.end())
  {
    CONSOLE_BRIDGE_logWarn("TaskComposerPluginFactory, tried to get task composer node '%s' that does not "
                           "exist!",
                           name.c_str());
    return nullptr;
  }

  return createTaskComposerNode(name, cm_it->second);
}

TaskComposerNode::UPtr
TaskComposerPluginFactory::createTaskComposerNode(const std::string& name,
                                                  const tesseract_common::PluginInfo& plugin_info) const
{
  try
  {
    auto it = node_factories_.find(plugin_info.class_name);
    if (it != node_factories_.end())
      return it->second->create(name, plugin_info.config, *this);

    auto plugin = plugin_loader_.instantiate<TaskComposerNodeFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    node_factories_[plugin_info.class_name] = plugin;
    return plugin->create(name, plugin_info.config, *this);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s', Details: %s", plugin_info.class_name.c_str(), e.what());
    return nullptr;
  }
}

void TaskComposerPluginFactory::saveConfig(const tesseract_common::fs::path& file_path) const
{
  YAML::Node config = getConfig();
  std::ofstream fout(file_path.string());
  fout << config;
}

YAML::Node TaskComposerPluginFactory::getConfig() const
{
  tesseract_common::TaskComposerPluginInfo tc_plugins;
  tc_plugins.search_paths = plugin_loader_.search_paths;
  tc_plugins.search_libraries = plugin_loader_.search_libraries;
  tc_plugins.executor_plugin_infos = executor_plugin_info_;
  tc_plugins.task_plugin_infos = task_plugin_info_;

  YAML::Node config;
  config[tesseract_common::TaskComposerPluginInfo::CONFIG_KEY] = tc_plugins;

  return config;
}
}  // namespace tesseract_planning
