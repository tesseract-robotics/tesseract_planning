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
#include <memory>
#include <set>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <boost_plugin_loader/fwd.h>
#include <boost_plugin_loader/macros.h>
#include <filesystem>

// clang-format off
#define TESSERACT_ADD_TASK_COMPOSER_EXECUTOR_PLUGIN(DERIVED_CLASS, ALIAS)                                                    \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, TaskExec)

#define TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(DERIVED_CLASS, ALIAS)                                                  \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, TaskNode)
// clang-format on

namespace YAML
{
class Node;
}

namespace tesseract_planning
{
class TaskComposerNode;
class TaskComposerExecutor;
class TaskComposerPluginFactory;

/** @brief Task Composer Node Factory class used by the TaskComposerServer for loading top level task to be called by
 * name */
class TaskComposerNodeFactory
{
public:
  using Ptr = std::shared_ptr<TaskComposerNodeFactory>;
  using ConstPtr = std::shared_ptr<const TaskComposerNodeFactory>;

  virtual ~TaskComposerNodeFactory() = default;

  virtual std::unique_ptr<TaskComposerNode> create(const std::string& name,
                                                   const YAML::Node& config,
                                                   const TaskComposerPluginFactory& plugin_factory) const = 0;

protected:
  static std::string getSection();
  friend class boost_plugin_loader::PluginLoader;
};

/** @brief Task Composer Executor Factory class used by the TaskComposerServer for loading executors to be called by
 * name */
class TaskComposerExecutorFactory
{
public:
  using Ptr = std::shared_ptr<TaskComposerExecutorFactory>;
  using ConstPtr = std::shared_ptr<const TaskComposerExecutorFactory>;

  virtual ~TaskComposerExecutorFactory() = default;

  virtual std::unique_ptr<TaskComposerExecutor> create(const std::string& name, const YAML::Node& config) const = 0;

protected:
  static std::string getSection();
  friend class boost_plugin_loader::PluginLoader;
};

class TaskComposerPluginFactory
{
public:
  using PluginInfoMap = std::map<std::string, tesseract_common::PluginInfo>;

  TaskComposerPluginFactory();
  ~TaskComposerPluginFactory();
  TaskComposerPluginFactory(const TaskComposerPluginFactory&) = delete;
  TaskComposerPluginFactory& operator=(const TaskComposerPluginFactory&) = delete;
  TaskComposerPluginFactory(TaskComposerPluginFactory&&) noexcept;
  TaskComposerPluginFactory& operator=(TaskComposerPluginFactory&&) noexcept;

  /**
   * @brief Load plugins from a configuration object
   * @param config The config object
   */
  TaskComposerPluginFactory(const tesseract_common::TaskComposerPluginInfo& config);

  /**
   * @brief Load plugins from yaml node
   * @param config The config node
   */
  TaskComposerPluginFactory(const YAML::Node& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from file path
   * @param config The config file path
   */
  TaskComposerPluginFactory(const std::filesystem::path& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from string
   * @param config The config string
   */
  TaskComposerPluginFactory(const std::string& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Loads plugins from a configuration object
   * @param config the config object
   */
  void loadConfig(const tesseract_common::TaskComposerPluginInfo& config);

  /**
   * @brief Load plugins from yaml node
   * @param config The config node
   */
  void loadConfig(const YAML::Node& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from file path
   * @param config The config file path
   */
  void loadConfig(const std::filesystem::path& config, const tesseract_common::ResourceLocator& locator);

  /**
   * @brief Load plugins from string
   * @param config The config string
   */
  void loadConfig(const std::string& config, const tesseract_common::ResourceLocator& locator);

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
  PluginInfoMap getTaskComposerExecutorPlugins() const;

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

  PluginInfoMap getTaskComposerNodePlugins() const;

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
  std::unique_ptr<TaskComposerExecutor> createTaskComposerExecutor(const std::string& name) const;

  /**
   * @brief Get task composer executor object given plugin info
   * @param name The name
   * @param plugin_info The plugin information to create task composer executor object
   */
  std::unique_ptr<TaskComposerExecutor>
  createTaskComposerExecutor(const std::string& name, const tesseract_common::PluginInfo& plugin_info) const;

  /**
   * @brief Get task composer node object given name
   * @details This looks for task composer node plugin info. If not found nullptr is returned.
   * @param name The name
   */
  std::unique_ptr<TaskComposerNode> createTaskComposerNode(const std::string& name) const;

  /**
   * @brief Get task composer node  object given plugin info
   * @param name The name
   * @param plugin_info The plugin information to task composer node  object
   */
  std::unique_ptr<TaskComposerNode> createTaskComposerNode(const std::string& name,
                                                           const tesseract_common::PluginInfo& plugin_info) const;

  /**
   * @brief Save the plugin information to a yaml config file
   * @param file_path The file path
   */
  void saveConfig(const std::filesystem::path& file_path) const;

  /**
   * @brief Get the plugin information config as a yaml node
   * @return The plugin information config yaml node/
   */
  YAML::Node getConfig() const;

private:
  struct Implementation;
  std::unique_ptr<Implementation> impl_;

  void loadConfig(const YAML::Node& config);
};
}  // namespace tesseract_planning
#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_FACTORY_H
