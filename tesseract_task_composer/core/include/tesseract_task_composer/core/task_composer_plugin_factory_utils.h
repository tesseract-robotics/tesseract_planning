/**
 * @file task_composer_plugin_factory_utils.h
 * @brief A task in the pipeline
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PLUGIN_FACTORY_UTILS_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PLUGIN_FACTORY_UTILS_H

#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

namespace tesseract_planning
{
template <typename TaskType>
class TaskComposerTaskFactory : public TaskComposerNodeFactory
{
public:
  TaskComposerNode::UPtr create(const std::string& name,
                                const YAML::Node& config,
                                const TaskComposerPluginFactory& plugin_factory) const override
  {
    return std::make_unique<TaskType>(name, config, plugin_factory);
  }
};

template <typename ExecutorType>
class TaskComposerExecutorFactoryImpl : public TaskComposerExecutorFactory
{
public:
  TaskComposerExecutor::UPtr create(const std::string& name, const YAML::Node& config) const override
  {
    return std::make_unique<ExecutorType>(name, config);
  }
};
}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PLUGIN_FACTORY_UTILS_H
