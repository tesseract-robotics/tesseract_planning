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

#include <tesseract_task_composer/planning/planning_task_composer_plugin_factories.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>

#include <tesseract_task_composer/planning/nodes/iterative_spline_parameterization_task.h>

namespace tesseract_planning
{
using IterativeSplineParameterizationTaskFactory = TaskComposerTaskFactory<IterativeSplineParameterizationTask>;

}  // namespace tesseract_planning

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(tesseract_planning::IterativeSplineParameterizationTaskFactory,
                                        IterativeSplineParameterizationTaskFactory)
