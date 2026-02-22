/**
 * @file task_composer_task_plugin_factory.cpp
 * @brief A task in the pipeline
 *
 * @author Levi Armstrong
 * @date July 29. 2022
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

#include <tesseract_task_composer/core/task_composer_task_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_pipeline.h>

#include <tesseract_task_composer/core/nodes/done_task.h>
#include <tesseract_task_composer/core/nodes/error_task.h>
#include <tesseract_task_composer/core/nodes/has_data_storage_entry_task.h>
#include <tesseract_task_composer/core/nodes/remap_task.h>
#include <tesseract_task_composer/core/nodes/start_task.h>
#include <tesseract_task_composer/core/nodes/sync_task.h>
#include <tesseract_task_composer/core/test_suite/test_task.h>

#include <boost_plugin_loader/macros.h>

namespace tesseract::task_composer
{
using DoneTaskFactory = TaskComposerTaskFactory<DoneTask>;
using ErrorTaskFactory = TaskComposerTaskFactory<ErrorTask>;
using HasDataStorageEntryTaskFactory = TaskComposerTaskFactory<HasDataStorageEntryTask>;
using RemapTaskFactory = TaskComposerTaskFactory<RemapTask>;
using StartTaskFactory = TaskComposerTaskFactory<StartTask>;
using SyncTaskFactory = TaskComposerTaskFactory<SyncTask>;
using GraphTaskFactory = TaskComposerTaskFactory<TaskComposerGraph>;
using PipelineTaskFactory = TaskComposerTaskFactory<TaskComposerPipeline>;

// LCOV_EXCL_START
PLUGIN_ANCHOR_IMPL(TaskComposerTaskFactoryAnchor)
// LCOV_EXCL_STOP

}  // namespace tesseract::task_composer

namespace tesseract::task_composer::test_suite
{
using TestTaskFactory = TaskComposerTaskFactory<TestTask>;
}

// clang-format off
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN( tesseract::task_composer::DoneTaskFactory, DoneTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN( tesseract::task_composer::ErrorTaskFactory, ErrorTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN( tesseract::task_composer::HasDataStorageEntryTaskFactory, HasDataStorageEntryTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN( tesseract::task_composer::RemapTaskFactory, RemapTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN( tesseract::task_composer::StartTaskFactory, StartTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN( tesseract::task_composer::SyncTaskFactory, SyncTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN( tesseract::task_composer::GraphTaskFactory, GraphTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN( tesseract::task_composer::PipelineTaskFactory, PipelineTaskFactory)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN( tesseract::task_composer::test_suite::TestTaskFactory, TestTaskFactory)
// clang-format on
