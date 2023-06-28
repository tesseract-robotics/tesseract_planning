/**
 * @file task_composer_executor_plugin_factory.h
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

#include <tesseract_task_composer/taskflow/taskflow_task_composer_plugin_factories.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>

namespace tesseract_planning
{
using TaskflowTaskComposerExecutorFactory = TaskComposerExecutorFactoryImpl<TaskflowTaskComposerExecutor>;
// LCOV_EXCL_START
TESSERACT_PLUGIN_ANCHOR_IMPL(TaskComposerTaskflowFactoriesAnchor)
// LCOV_EXCL_STOP
}  // namespace tesseract_planning

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_EXECUTOR_PLUGIN(tesseract_planning::TaskflowTaskComposerExecutorFactory,
                                            TaskflowTaskComposerExecutorFactory)
