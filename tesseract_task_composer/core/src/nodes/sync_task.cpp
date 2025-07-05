/**
 * @file sync_task.cpp
 *
 * @author Levi Armstrong
 * @date August 13, 2023
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/property_tree.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/nodes/sync_task.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_schema.h>

namespace tesseract_planning
{
SyncTask::SyncTask(std::string name) : TaskComposerTask(std::move(name), TaskComposerNodePorts{}, false) {}
SyncTask::SyncTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), TaskComposerNodePorts{}, config)
{
  if (conditional_)
    throw std::runtime_error("SyncTask, config is_conditional should not be true");
}

tesseract_common::PropertyTree SyncTask::getSchema() const
{
  using namespace tesseract_common;

  PropertyTree schema;
  schema.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  schema.setAttribute(property_attribute::REQUIRED, true);
  schema.setAttribute(property_attribute::TASK_NAME, "SyncTask");
  schema.setAttribute(property_attribute::FACTORY_NAME, "SyncTaskFactory");
  schema.setAttribute(property_attribute::DOC, "A syncronization task");
  std::map<int, std::string> return_options;
  return_options[1] = "Successful";
  schema.setAttribute("return_options", YAML::Node(return_options));

  /** @todo These should be removed completely for this task */
  addConditionalProperty(schema, false);
  addTriggerAbortProperty(schema);

  return schema;
}

TaskComposerNodeInfo SyncTask::runImpl(TaskComposerContext& /*context*/,
                                       OptionalTaskComposerExecutor /*executor*/) const
{
  TaskComposerNodeInfo info(*this);
  info.color = "green";
  info.return_value = 1;
  info.status_code = 1;
  info.status_message = "Successful";
  return info;
}

bool SyncTask::operator==(const SyncTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool SyncTask::operator!=(const SyncTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void SyncTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::SyncTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::SyncTask)
