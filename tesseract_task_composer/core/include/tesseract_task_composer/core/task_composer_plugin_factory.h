/**
 * @file task_composer_plugin_factory.h
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_FACTORY_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_FACTORY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <map>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_common/plugin_loader.h>

// clang-format off
#define TESSERACT_ADD_TASK_COMPOSER_EXECUTOR_PLUGIN(DERIVED_CLASS, ALIAS)                                                    \
  TESSERACT_ADD_PLUGIN_SECTIONED(DERIVED_CLASS, ALIAS, TaskExec)

#define TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(DERIVED_CLASS, ALIAS)                                                  \
  TESSERACT_ADD_PLUGIN_SECTIONED(DERIVED_CLASS, ALIAS, TaskNode)
// clang-format on

namespace tesseract_planning
{
class TaskComposerPluginFactory;

/** @brief Task Composer Node Factory class used by the TaskComposerServer for loading top level task to be called by
 * name */
class TaskComposerNodeFactory
{
public:
  using Ptr = std::shared_ptr<TaskComposerNodeFactory>;
  using ConstPtr = std::shared_ptr<const TaskComposerNodeFactory>;

  virtual ~TaskComposerNodeFactory() = default;

  virtual TaskComposerNode::UPtr create(const std::string& name,
                                        const YAML::Node& config,
                                        const TaskComposerPluginFactory& plugin_factory) const = 0;

protected:
  static const std::string SECTION_NAME;
  friend class PluginLoader;
};

/** @brief Task Composer Executor Factory class used by the TaskComposerServer for loading executors to be called by
 * name */
class TaskComposerExecutorFactory
{
public:
  using Ptr = std::shared_ptr<TaskComposerExecutorFactory>;
  using ConstPtr = std::shared_ptr<const TaskComposerExecutorFactory>;

  virtual ~TaskComposerExecutorFactory() = default;

  virtual TaskComposerExecutor::UPtr create(const std::string& name, const YAML::Node& config) const = 0;

protected:
  static const std::string SECTION_NAME;
  friend class PluginLoader;
};

class TaskComposerPluginFactory
{
public:
  TaskComposerPluginFactory();
  ~TaskComposerPluginFactory();
  TaskComposerPluginFactory(const TaskComposerPluginFactory&) = default;
  TaskComposerPluginFactory& operator=(const TaskComposerPluginFactory&) = default;
  TaskComposerPluginFactory(TaskComposerPluginFactory&&) = default;
  TaskComposerPluginFactory& operator=(TaskComposerPluginFactory&&) = default;

  /**
   * @brief Load plugins from a configuration object
   * @param config The config object
   */
  TaskComposerPluginFactory(const tesseract_common::TaskComposerPluginInfo& config);

  /**
   * @brief Load plugins from yaml node
   * @param config The config node
   */
  TaskComposerPluginFactory(const YAML::Node& config);

  /**
   * @brief Load plugins from file path
   * @param config The config file path
   */
  TaskComposerPluginFactory(const tesseract_common::fs::path& config);

  /**
   * @brief Load plugins from string
   * @param config The config string
   */
  TaskComposerPluginFactory(const std::string& config);

  /**
   * @brief Loads plugins from a configuration object
   * @param config the config object
   */
  void loadConfig(const tesseract_common::TaskComposerPluginInfo& config);

  /**
   * @brief Load plugins from yaml node
   * @param config The config node
   */
  void loadConfig(const YAML::Node& config);

  /**
   * @brief Load plugins from file path
   * @param config The config file path
   */
  void loadConfig(const tesseract_common::fs::path& config);

  /**
   * @brief Load plugins from string
   * @param config The config string
   */
  void loadConfig(const std::string& config);

  /**
   * @brief Add location for the plugin loader to search
   * @param path The full path to the directory
   */
  void addSearchPath(const std::string& path);

  /**
   * @brief Get the plugin search paths
   * @return The search paths
   */
  std::set<std::string> getSearchPaths() const;

  /**
   * @brief Clear the search paths
   *
   */
  void clearSearchPaths();

  /**
   * @brief Add a library to search for plugin name
   * @param library_name The library name without the prefix or suffix
   */
  void addSearchLibrary(const std::string& library_name);

  /**
   * @brief Get the plugin search libraries
   * @return The search libraries
   */
  std::set<std::string> getSearchLibraries() const;

  /**
   * @brief Clean the search libraries
   *
   */
  void clearSearchLibraries();

  /**
   * @brief Add a task composer executor plugin
   * @param name The name
   * @param plugin_info The plugin information
   */
  void addTaskComposerExecutorPlugin(const std::string& name, tesseract_common::PluginInfo plugin_info);

  /**
   * @brief Check if it has task composer executor plugins
   * @return True if task composer executor PluginInfoMap is not empty, otherwise fale
   */
  bool hasTaskComposerExecutorPlugins() const;

  /**
   * @brief Get the map of task composer executor plugins
   * @return A map of plugins
   */
  tesseract_common::PluginInfoMap getTaskComposerExecutorPlugins() const;

  /**
   * @brief Remove task composer executor plugin
   * @param name The name of the executor to remove
   */
  void removeTaskComposerExecutorPlugin(const std::string& name);

  /**
   * @brief Set a default task composer executor
   * @param name The name
   */
  void setDefaultTaskComposerExecutorPlugin(const std::string& name);

  /**
   * @brief Get the default task composer executor
   * @return The default task composer executor
   */
  std::string getDefaultTaskComposerExecutorPlugin() const;

  /**
   * @brief Add a task composer node plugin
   * @param name The name
   * @param plugin_info The plugin information
   */
  void addTaskComposerNodePlugin(const std::string& name, tesseract_common::PluginInfo plugin_info);

  /**
   * @brief Check if it has task composer node plugins
   * @return True if task composer node PluginInfoMap is not empty, otherwise fale
   */
  bool hasTaskComposerNodePlugins() const;

  /**
   * @brief Get the map of task composer node plugins
   * @return A map of plugins
   */
  tesseract_common::PluginInfoMap getTaskComposerNodePlugins() const;

  /**
   * @brief Remove task composer node plugin
   * @param name The name of the task composer node to remove
   */
  void removeTaskComposerNodePlugin(const std::string& name);

  /**
   * @brief Set a default task composer node
   * @param name The name
   */
  void setDefaultTaskComposerNodePlugin(const std::string& name);

  /**
   * @brief Get the default task composer node
   * @return The default task composer node name
   */
  std::string getDefaultTaskComposerNodePlugin() const;

  /**
   * @brief Get task composer executor object given name
   * @details This looks for task composer executor plugin info. If not found nullptr is returned.
   * @param name The name
   */
  TaskComposerExecutor::UPtr createTaskComposerExecutor(const std::string& name) const;

  /**
   * @brief Get task composer executor object given plugin info
   * @param name The name
   * @param plugin_info The plugin information to create task composer executor object
   */
  TaskComposerExecutor::UPtr createTaskComposerExecutor(const std::string& name,
                                                        const tesseract_common::PluginInfo& plugin_info) const;

  /**
   * @brief Get task composer node object given name
   * @details This looks for task composer node plugin info. If not found nullptr is returned.
   * @param name The name
   */
  TaskComposerNode::UPtr createTaskComposerNode(const std::string& name) const;

  /**
   * @brief Get task composer node  object given plugin info
   * @param name The name
   * @param plugin_info The plugin information to task composer node  object
   */
  TaskComposerNode::UPtr createTaskComposerNode(const std::string& name,
                                                const tesseract_common::PluginInfo& plugin_info) const;

  /**
   * @brief Save the plugin information to a yaml config file
   * @param file_path The file path
   */
  void saveConfig(const boost::filesystem::path& file_path) const;

  /**
   * @brief Get the plugin information config as a yaml node
   * @return The plugin information config yaml node/
   */
  YAML::Node getConfig() const;

private:
  mutable std::map<std::string, TaskComposerExecutorFactory::Ptr> executor_factories_;
  mutable std::map<std::string, TaskComposerNodeFactory::Ptr> node_factories_;
  tesseract_common::PluginInfoContainer executor_plugin_info_;
  tesseract_common::PluginInfoContainer task_plugin_info_;
  tesseract_common::PluginLoader plugin_loader_;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_FACTORY_H
