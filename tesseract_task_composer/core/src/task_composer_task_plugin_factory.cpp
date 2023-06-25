/**
 * @file task_composer_task_plugin_factory.cpp
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

#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_pipeline.h>

#include <tesseract_task_composer/core/nodes/abort_task.h>
#include <tesseract_task_composer/core/nodes/done_task.h>
#include <tesseract_task_composer/core/nodes/error_task.h>
#include <tesseract_task_composer/core/nodes/start_task.h>

#include <tesseract_task_composer/core/factories/task_composer_task_plugin_factory.h>

namespace tesseract_planning
{
using AbortTaskFactory = TaskComposerTaskFactory<AbortTask>;
using DoneTaskFactory = TaskComposerTaskFactory<DoneTask>;
using ErrorTaskFactory = TaskComposerTaskFactory<ErrorTask>;
using StartTaskFactory = TaskComposerTaskFactory<StartTask>;
using GraphTaskFactory = TaskComposerTaskFactory<TaskComposerGraph>;
using PipelineTaskFactory = TaskComposerTaskFactory<TaskComposerPipeline>;

TESSERACT_PLUGIN_ANCHOR_IMPL(TaskComposerTaskFactoryAnchor);
}  // namespace tesseract_planning

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::AbortTaskFactory, AbortTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::DoneTaskFactory, DoneTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::ErrorTaskFactory, ErrorTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::StartTaskFactory, StartTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::GraphTaskFactory, GraphTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::PipelineTaskFactory, PipelineTaskFactory)
