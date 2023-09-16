/**
 * @file task_composer_task.cpp
 * @brief A task
 *
 * @author Levi Armstrong
 * @date July 29. 2022
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
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_common/timer.h>

namespace tesseract_planning
{
TaskComposerTask::TaskComposerTask(std::string name) : TaskComposerTask(std::move(name), true) {}
TaskComposerTask::TaskComposerTask(std::string name, bool conditional)
  : TaskComposerNode(std::move(name), TaskComposerNodeType::TASK, conditional)
{
}

TaskComposerTask::TaskComposerTask(std::string name, const YAML::Node& config)
  : TaskComposerNode(std::move(name), TaskComposerNodeType::TASK, config)
{
}

int TaskComposerTask::run(const TaskComposerContext::Ptr& context, OptionalTaskComposerExecutor executor) const
{
  if (context->isAborted())
  {
    auto info = std::make_unique<TaskComposerNodeInfo>(*this);
    info->input_keys = input_keys_;
    info->output_keys = output_keys_;
    info->return_value = 0;
    info->color = "white";
    info->message = "Aborted";
    info->aborted_ = true;
    context->getTaskInfos().addInfo(std::move(info));
    return 0;
  }

  tesseract_common::Timer timer;
  TaskComposerNodeInfo::UPtr results;
  timer.start();
  try
  {
    results = runImpl(context, executor);
  }
  catch (const std::exception& e)
  {
    results = std::make_unique<TaskComposerNodeInfo>(*this);
    results->color = "red";
    results->message = "Exception thrown: " + std::string(e.what());
    results->return_value = 0;
  }
  timer.stop();
  results->input_keys = input_keys_;
  results->output_keys = output_keys_;
  results->elapsed_time = timer.elapsedSeconds();

  int value = results->return_value;
  assert(value >= 0);
  context->getTaskInfos().addInfo(std::move(results));
  return value;
}

bool TaskComposerTask::operator==(const TaskComposerTask& rhs) const { return (TaskComposerNode::operator==(rhs)); }

// LCOV_EXCL_START
bool TaskComposerTask::operator!=(const TaskComposerTask& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void TaskComposerTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNode);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerTask)
