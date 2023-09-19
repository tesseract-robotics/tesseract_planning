/**
 * @file remap_task.h
 *
 * @author Levi Armstrong
 * @date July 13, 2023
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/nodes/remap_task.h>

namespace tesseract_planning
{
RemapTask::RemapTask() : TaskComposerTask("RemapTask", false) {}
RemapTask::RemapTask(std::string name, std::map<std::string, std::string> remap, bool copy, bool is_conditional)
  : TaskComposerTask(std::move(name), is_conditional), remap_(std::move(remap)), copy_(copy)
{
  if (remap_.empty())
    throw std::runtime_error("RemapTask, remap should not be empty!");
}
RemapTask::RemapTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  if (!input_keys_.empty())
    throw std::runtime_error("RemapTask, input_keys should be empty!");

  if (!output_keys_.empty())
    throw std::runtime_error("RemapTask, output_keys should be empty!");

  if (YAML::Node n = config["remap"])
    remap_ = n.as<std::map<std::string, std::string>>();
  else
    throw std::runtime_error("RemapTask missing config key: 'remap'");

  if (YAML::Node n = config["copy"])
    copy_ = n.as<bool>();
}

TaskComposerNodeInfo::UPtr RemapTask::runImpl(TaskComposerContext& context,
                                              OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  if (context.data_storage->remapData(remap_, copy_))
  {
    info->color = "green";
    info->return_value = 1;
    info->message = "Successful";
  }
  else
  {
    info->color = "red";
    info->return_value = 0;
    info->message = "Failed to remap data.";
  }
  return info;
}

bool RemapTask::operator==(const RemapTask& rhs) const
{
  bool equal = true;
  equal &= (remap_ == rhs.remap_);
  equal &= (copy_ == rhs.copy_);
  equal &= TaskComposerNode::operator==(rhs);
  return equal;
}
bool RemapTask::operator!=(const RemapTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void RemapTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
  ar& boost::serialization::make_nvp("remap_data", remap_);
  ar& boost::serialization::make_nvp("copy", copy_);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RemapTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RemapTask)
