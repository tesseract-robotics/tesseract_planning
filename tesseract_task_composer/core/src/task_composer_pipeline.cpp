/**
 * @file task_composer_pipeline.cpp
 * @brief A task pipeline
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
#include <tesseract_common/stopwatch.h>
#include <tesseract_common/serialization.h>
#include <boost/uuid/uuid_io.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_pipeline.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

namespace tesseract_planning
{
TaskComposerPipeline::TaskComposerPipeline(std::string name) : TaskComposerPipeline(std::move(name), true) {}
TaskComposerPipeline::TaskComposerPipeline(std::string name, bool conditional)
  : TaskComposerGraph(std::move(name), TaskComposerNodeType::PIPELINE, conditional)
{
}
TaskComposerPipeline::TaskComposerPipeline(std::string name,
                                           const YAML::Node& config,
                                           const TaskComposerPluginFactory& plugin_factory)
  : TaskComposerGraph(std::move(name), TaskComposerNodeType::PIPELINE, config, plugin_factory)
{
}

TaskComposerNodeInfo TaskComposerPipeline::runImpl(TaskComposerContext& context,
                                                   OptionalTaskComposerExecutor executor) const
{
  if (terminals_.empty())
    throw std::runtime_error("TaskComposerPipeline, with name '" + name_ + "' does not have terminals!");

  tesseract_common::Stopwatch stopwatch;
  stopwatch.start();
  boost::uuids::uuid root_node = getRootNode();

  if (root_node.is_nil())
    throw std::runtime_error("TaskComposerPipeline, with name '" + name_ + "' does not have a root node!");

  // Create local data storage for graph
  TaskComposerDataStorage::Ptr parent_data_storage = getDataStorage(context);

  // Create new data storage and copy input data
  auto data_storage = std::make_shared<TaskComposerDataStorage>(uuid_str_);
  data_storage->copyData(*parent_data_storage, input_keys_);

  // Store the new data storage for access by child nodes
  context.data_storage->setData(uuid_str_, data_storage);

  // Run
  runRecursive(*(nodes_.at(root_node)), context, executor);

  // Copy output data to parent data storage
  parent_data_storage->copyData(*data_storage, output_keys_);

  for (std::size_t i = 0; i < terminals_.size(); ++i)
  {
    auto node_info = context.task_infos.getInfo(terminals_[i]);
    if (node_info.has_value())
    {
      stopwatch.stop();
      TaskComposerNodeInfo info(*this);
      info.return_value = static_cast<int>(i);
      info.color = node_info->color;
      info.status_code = node_info->status_code;
      info.status_message = node_info->status_message;
      info.elapsed_time = stopwatch.elapsedSeconds();
      return info;
    }
  }

  throw std::runtime_error("TaskComposerPipeline, with name '" + name_ +
                           "' has no node info for any of the leaf nodes!");
}

void TaskComposerPipeline::runRecursive(const TaskComposerNode& node,
                                        TaskComposerContext& context,
                                        OptionalTaskComposerExecutor executor) const
{
  if (node.getType() == TaskComposerNodeType::NODE)
    throw std::runtime_error("TaskComposerPipeline, unsupported node type TaskComposerNodeType::NODE");

  int rv = node.run(context, executor);
  if (node.isConditional())
  {
    const auto& edge = node.getOutboundEdges().at(static_cast<std::size_t>(rv));
    runRecursive(*nodes_.at(edge), context, executor);
  }
  else
  {
    if (node.getOutboundEdges().size() > 1)
      throw std::runtime_error("TaskComposerPipeline, non conditional task can only have one out bound edge. Name: "
                               "'" +
                               name_ + "'");
    for (const auto& edge : node.getOutboundEdges())
      runRecursive(*(nodes_.at(edge)), context, executor);
  }
}

bool TaskComposerPipeline::operator==(const TaskComposerPipeline& rhs) const
{
  return (TaskComposerGraph::operator==(rhs));
}
bool TaskComposerPipeline::operator!=(const TaskComposerPipeline& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerPipeline::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerPipeline)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerPipeline)
