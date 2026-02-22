/**
 * @file sync_task.h
 *
 * @author Levi Armstrong
 * @date August 13, 2023
 *
 * @copyright Copyright (c) 2023, Plectix Robotics
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
#ifndef TESSERACT_TASK_COMPOSER_SYNC_TASK_H
#define TESSERACT_TASK_COMPOSER_SYNC_TASK_H

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract::task_composer
{
class TaskComposerPluginFactory;

/**
 * @brief This is used as a synchronization point in your task graph
 *
 * The actual synchronization is something that the executor will inherently do based on the graph structure but it
 * needs a actual task so you can connect all parallel tasks to. This way when it gets to this task the executor will
 * not proceed until all preceding tasks are finished.
 *
 * @note This task has no inputs or output and is not conditional
 */
class SyncTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<SyncTask>;
  using ConstPtr = std::shared_ptr<const SyncTask>;
  using UPtr = std::unique_ptr<SyncTask>;
  using ConstUPtr = std::unique_ptr<const SyncTask>;

  explicit SyncTask(std::string name = "SyncTask");
  explicit SyncTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& plugin_factory);
  ~SyncTask() override = default;

private:
  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract::task_composer

#endif  // TESSERACT_TASK_COMPOSER_SYNC_TASK_H
