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
#include <boost/serialization/map.hpp>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/property_tree.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/nodes/remap_task.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_schema.h>

namespace tesseract_planning
{
const std::string RemapTask::INOUT_KEYS_PORT = "keys";

RemapTask::RemapTask() : TaskComposerTask("RemapTask", RemapTask::ports(), false) {}
RemapTask::RemapTask(std::string name, const std::map<std::string, std::string>& remap, bool copy, bool is_conditional)
  : TaskComposerTask(std::move(name), RemapTask::ports(), is_conditional), copy_(copy)
{
  if (remap.empty())
    throw std::runtime_error("RemapTask, remap should not be empty!");

  std::vector<std::string> ikeys;
  std::vector<std::string> okeys;
  ikeys.reserve(remap.size());
  okeys.reserve(remap.size());
  for (const auto& pair : remap)
  {
    ikeys.push_back(pair.first);
    okeys.push_back(pair.second);
  }

  input_keys_.add(INOUT_KEYS_PORT, ikeys);
  output_keys_.add(INOUT_KEYS_PORT, okeys);
  validatePorts();
}
RemapTask::RemapTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), RemapTask::ports(), config)
{
  if (input_keys_.get<std::vector<std::string>>(INOUT_KEYS_PORT).size() !=
      output_keys_.get<std::vector<std::string>>(INOUT_KEYS_PORT).size())
    throw std::runtime_error("RemapTask, input and ouput port 'keys' must be same size");

  if (YAML::Node n = config["copy"])
    copy_ = n.as<bool>();
}

tesseract_common::PropertyTree RemapTask::getSchema() const
{
  using namespace tesseract_common;

  PropertyTree schema;
  schema.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  schema.setAttribute(property_attribute::REQUIRED, true);
  schema.setAttribute(property_attribute::TASK_NAME, "RemapTask");
  schema.setAttribute(property_attribute::FACTORY_NAME, "RemapTaskFactory");
  schema.setAttribute(property_attribute::DOC, "A task to remap data stored in data storage from one key to another");

  std::map<int, std::string> return_options;
  return_options[0] = "Error";
  return_options[0] = "Successful";
  schema.setAttribute("return_options", YAML::Node(return_options));

  addConditionalProperty(schema, false);
  addTriggerAbortProperty(schema);
  {
    auto& prop = schema["copy"];
    prop.setAttribute(property_attribute::TYPE, property_type::BOOL);
    prop.setAttribute(property_attribute::DEFAULT, false);
    prop.setAttribute(property_attribute::DOC, "Indicate if data should be copied, otherwise it is moved");
    prop.setAttribute(property_attribute::REQUIRED, false);
  }

  PropertyTree& inputs = addInputsProperty(schema);
  {
    auto& prop = inputs[INOUT_KEYS_PORT];
    prop.setAttribute(property_attribute::TYPE, property_type::createList(property_type::STRING));
    prop.setAttribute(property_attribute::DOC, "A list of keys in data storage to copy/move");
    prop.setAttribute(property_attribute::REQUIRED, true);
  }

  PropertyTree& outputs = addInputsProperty(schema);
  {
    auto& prop = outputs[INOUT_KEYS_PORT];
    prop.setAttribute(property_attribute::TYPE, property_type::createList(property_type::STRING));
    prop.setAttribute(property_attribute::DOC, "A list of keys in data storage to copy/move");
    prop.setAttribute(property_attribute::REQUIRED, true);
  }

  return schema;
}

TaskComposerNodePorts RemapTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INOUT_KEYS_PORT] = TaskComposerNodePorts::MULTIPLE;
  ports.output_required[INOUT_KEYS_PORT] = TaskComposerNodePorts::MULTIPLE;
  return ports;
}

TaskComposerNodeInfo RemapTask::runImpl(TaskComposerContext& context, OptionalTaskComposerExecutor /*executor*/) const
{
  TaskComposerNodeInfo info(*this);
  const auto& ikeys = input_keys_.get<std::vector<std::string>>(INOUT_KEYS_PORT);
  const auto& okeys = output_keys_.get<std::vector<std::string>>(INOUT_KEYS_PORT);
  std::map<std::string, std::string> remapping;
  for (std::size_t i = 0; i < ikeys.size(); ++i)
    remapping[ikeys[i]] = okeys[i];

  if (context.data_storage->remapData(remapping, copy_))
  {
    info.color = "green";
    info.return_value = 1;
    info.status_code = 1;
    info.status_message = "Successful";
  }
  else
  {
    info.color = "red";
    info.return_value = 0;
    info.status_code = 0;
    info.status_message = "Failed to remap data.";
  }
  return info;
}

bool RemapTask::operator==(const RemapTask& rhs) const
{
  bool equal = true;
  equal &= (copy_ == rhs.copy_);
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool RemapTask::operator!=(const RemapTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void RemapTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
  ar& boost::serialization::make_nvp("copy", copy_);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RemapTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RemapTask)
