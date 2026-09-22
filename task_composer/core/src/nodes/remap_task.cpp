/**
 * @file remap_task.h
 *
 * @author Levi Armstrong
 * @date July 13, 2023
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/task_composer/nodes/remap_task.h>
#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/task_composer/task_composer_data_storage.h>
#include <tesseract/task_composer/task_composer_node_info.h>
#include <tesseract/common/property_tree.h>

namespace tesseract::task_composer
{

RemapTask::RemapTask() : TaskComposerTask("RemapTask", RemapTask::ports(), false) {}
RemapTask::RemapTask(std::string name, const std::map<std::string, std::string>& remap, bool copy, bool is_conditional)
  : TaskComposerTask(std::move(name), RemapTask::ports(), is_conditional), copy_(copy)
{
  if (remap.empty())
    throw std::runtime_error("RemapTask, remap should not be empty!");

  std::vector<std::string> input_storage_keys;
  std::vector<std::string> output_storage_keys;
  input_storage_keys.reserve(remap.size());
  output_storage_keys.reserve(remap.size());
  for (const auto& pair : remap)
  {
    input_storage_keys.push_back(pair.first);
    output_storage_keys.push_back(pair.second);
  }

  input_port_mappings_.set(INOUT_STORAGE_KEYS_PORT, input_storage_keys);
  output_port_mappings_.set(INOUT_STORAGE_KEYS_PORT, output_storage_keys);
  setPortMappings(input_port_mappings_, output_port_mappings_);
}
RemapTask::RemapTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), RemapTask::ports(), config)
{
  if (input_port_mappings_.multiple(INOUT_STORAGE_KEYS_PORT).size() !=
      output_port_mappings_.multiple(INOUT_STORAGE_KEYS_PORT).size())
    throw std::runtime_error("RemapTask input and output storage-key mappings must have the same size");

  if (YAML::Node n = config["copy"])
    copy_ = n.as<bool>();
}

const TaskComposerNodePorts& RemapTask::ports()
{
  static const TaskComposerNodePorts ports = []() {
    TaskComposerNodePorts ports;
    ports.addRequiredInput(INOUT_STORAGE_KEYS_PORT, TaskComposerNodePorts::Cardinality::MULTIPLE);
    ports.addRequiredOutput(INOUT_STORAGE_KEYS_PORT, TaskComposerNodePorts::Cardinality::MULTIPLE);
    return ports;
  }();
  return ports;
}

TaskComposerNodeInfo RemapTask::runImpl(TaskComposerContext& context, OptionalTaskComposerExecutor /*executor*/) const
{
  TaskComposerNodeInfo info(*this);
  const auto& input_storage_keys = input_port_mappings_.multiple(INOUT_STORAGE_KEYS_PORT);
  const auto& output_storage_keys = output_port_mappings_.multiple(INOUT_STORAGE_KEYS_PORT);
  std::map<std::string, std::string> remapping;
  for (std::size_t i = 0; i < input_storage_keys.size(); ++i)
    remapping[input_storage_keys[i]] = output_storage_keys[i];

  // Get local data storage
  TaskComposerDataStorage::Ptr data_storage = getDataStorage(context);

  if (data_storage->remapData(remapping, copy_))
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

tesseract::common::PropertyTree RemapTask::schema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .compose(TaskComposerTask::schema(RemapTask::ports()))
      .boolean("copy").defaultVal(false).done()
      .build();
  // clang-format on
}
}  // namespace tesseract::task_composer
