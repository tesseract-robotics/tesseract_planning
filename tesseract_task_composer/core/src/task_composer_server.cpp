/**
 * @file task_composer_server.h
 * @brief A task server
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
#include <console_bridge/console.h>
#include <tesseract_common/plugin_info.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_server.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

namespace tesseract_planning
{
TaskComposerServer::TaskComposerServer() : plugin_factory_(std::make_shared<TaskComposerPluginFactory>()) {}

void TaskComposerServer::loadConfig(const YAML::Node& config)
{
  plugin_factory_->loadConfig(config);
  loadPlugins();
}

void TaskComposerServer::loadConfig(const tesseract_common::fs::path& config)
{
  plugin_factory_->loadConfig(config);
  loadPlugins();
}

void TaskComposerServer::loadConfig(const std::string& config)
{
  plugin_factory_->loadConfig(config);
  loadPlugins();
}

void TaskComposerServer::addExecutor(const std::shared_ptr<TaskComposerExecutor>& executor)
{
  executors_[executor->getName()] = executor;
}

std::shared_ptr<TaskComposerExecutor> TaskComposerServer::getExecutor(const std::string& name)
{
  auto it = executors_.find(name);
  if (it == executors_.end())
    throw std::runtime_error("Executor with name '" + name + "' does not exist!");

  return it->second;
}

bool TaskComposerServer::hasExecutor(const std::string& name) const
{
  return (executors_.find(name) != executors_.end());
}

std::vector<std::string> TaskComposerServer::getAvailableExecutors() const
{
  std::vector<std::string> executors;
  executors.reserve(executors_.size());
  for (const auto& executor : executors_)
    executors.push_back(executor.first);

  return executors;
}

void TaskComposerServer::addTask(std::unique_ptr<TaskComposerNode> task)
{
  if (tasks_.find(task->getName()) != tasks_.end())
    CONSOLE_BRIDGE_logDebug("Task %s already exist so replacing with new task.", task->getName().c_str());

  tasks_[task->getName()] = std::move(task);
}

const TaskComposerNode& TaskComposerServer::getTask(const std::string& name)
{
  auto it = tasks_.find(name);
  if (it == tasks_.end())
    throw std::runtime_error("Task with name '" + name + "' does not exist!");

  return *(it->second);
}

bool TaskComposerServer::hasTask(const std::string& name) const { return (tasks_.find(name) != tasks_.end()); }

std::vector<std::string> TaskComposerServer::getAvailableTasks() const
{
  std::vector<std::string> tasks;
  tasks.reserve(tasks_.size());
  for (const auto& task : tasks_)
    tasks.push_back(task.first);

  return tasks;
}

std::unique_ptr<TaskComposerFuture> TaskComposerServer::run(const std::string& task_name,
                                                            std::shared_ptr<TaskComposerDataStorage> data_storage,
                                                            bool dotgraph,
                                                            const std::string& executor_name)
{
  auto e_it = executors_.find(executor_name);
  if (e_it == executors_.end())
    throw std::runtime_error("Executor with name '" + executor_name + "' does not exist!");

  auto t_it = tasks_.find(task_name);
  if (t_it == tasks_.end())
    throw std::runtime_error("Task with name '" + task_name + "' does not exist!");

  data_storage->setName(task_name);
  return e_it->second->run(*t_it->second, std::move(data_storage), dotgraph);
}

std::unique_ptr<TaskComposerFuture> TaskComposerServer::run(const TaskComposerNode& node,
                                                            std::shared_ptr<TaskComposerDataStorage> data_storage,
                                                            bool dotgraph,
                                                            const std::string& executor_name)
{
  auto it = executors_.find(executor_name);
  if (it == executors_.end())
    throw std::runtime_error("Executor with name '" + executor_name + "' does not exist!");

  data_storage->setName(node.getName());
  return it->second->run(node, std::move(data_storage), dotgraph);
}

long TaskComposerServer::getWorkerCount(const std::string& name) const
{
  auto it = executors_.find(name);
  if (it == executors_.end())
    throw std::runtime_error("Executor with name '" + name + "' does not exist!");

  return it->second->getWorkerCount();
}

long TaskComposerServer::getTaskCount(const std::string& name) const
{
  auto it = executors_.find(name);
  if (it == executors_.end())
    throw std::runtime_error("Executor with name '" + name + "' does not exist!");

  return it->second->getTaskCount();
}

void TaskComposerServer::loadPlugins()
{
  auto executor_plugins = plugin_factory_->getTaskComposerExecutorPlugins();
  for (const auto& executor_plugin : executor_plugins)
  {
    TaskComposerExecutor::UPtr e = plugin_factory_->createTaskComposerExecutor(executor_plugin.first);
    if (e != nullptr)
      addExecutor(std::move(e));
    else
      CONSOLE_BRIDGE_logError("TaskComposerServer, failed to create executor '%s'", executor_plugin.first.c_str());
  }

  auto task_plugins = plugin_factory_->getTaskComposerNodePlugins();
  for (const auto& task_plugin : task_plugins)
  {
    std::unique_ptr<TaskComposerNode> t = plugin_factory_->createTaskComposerNode(task_plugin.first);
    if (t != nullptr)
      addTask(std::move(t));
    else
      CONSOLE_BRIDGE_logError("TaskComposerServer, failed to create task '%s'", task_plugin.first.c_str());
  }
}
}  // namespace tesseract_planning
