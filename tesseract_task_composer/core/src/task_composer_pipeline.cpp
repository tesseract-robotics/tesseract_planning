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
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/core/task_composer_pipeline.h>
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

int TaskComposerPipeline::run(TaskComposerContext& context, OptionalTaskComposerExecutor executor) const
{
  auto start_time = std::chrono::system_clock::now();
  if (context.isAborted())
  {
    auto info = std::make_unique<TaskComposerNodeInfo>(*this);
    info->start_time = start_time;
    info->input_keys = input_keys_;
    info->output_keys = output_keys_;
    info->return_value = 0;
    info->color = "white";
    info->message = "Aborted";
    info->aborted_ = true;
    context.task_infos.addInfo(std::move(info));
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
  results->start_time = start_time;
  results->elapsed_time = timer.elapsedSeconds();

  int value = results->return_value;
  assert(value >= 0);
  context.task_infos.addInfo(std::move(results));
  return value;
}

TaskComposerNodeInfo::UPtr TaskComposerPipeline::runImpl(TaskComposerContext& context,
                                                         OptionalTaskComposerExecutor executor) const
{
  if (terminals_.empty())
    throw std::runtime_error("TaskComposerPipeline, with name '" + name_ + "' does not have terminals!");

  tesseract_common::Timer timer;
  timer.start();
  boost::uuids::uuid root_node{};
  for (const auto& pair : nodes_)
  {
    if (pair.second->getInboundEdges().empty())
    {
      root_node = pair.first;
      break;
    }
  }

  if (root_node.is_nil())
    throw std::runtime_error("TaskComposerPipeline, with name '" + name_ + "' does not have a root node!");

  runRecursive(*(nodes_.at(root_node)), context, executor);

  for (std::size_t i = 0; i < terminals_.size(); ++i)
  {
    auto node_info = context.task_infos.getInfo(terminals_[i]);
    if (node_info != nullptr)
    {
      timer.stop();
      auto info = std::make_unique<TaskComposerNodeInfo>(*this);
      info->input_keys = input_keys_;
      info->output_keys = output_keys_;
      info->return_value = static_cast<int>(i);
      info->color = node_info->color;
      info->message = node_info->message;
      info->elapsed_time = timer.elapsedSeconds();
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
  if (node.getType() == TaskComposerNodeType::GRAPH)
    throw std::runtime_error("TaskComposerPipeline, does not support GRAPH node types. Name: '" + name_ + "'");

  if (node.getType() == TaskComposerNodeType::TASK)
  {
    const auto& task = static_cast<const TaskComposerTask&>(node);
    int rv = task.run(context, executor);
    if (task.isConditional())
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
  else
  {
    const auto& pipeline = static_cast<const TaskComposerPipeline&>(node);
    int rv = pipeline.run(context, executor);
    if (pipeline.isConditional())
    {
      const auto& edge = node.getOutboundEdges().at(static_cast<std::size_t>(rv));
      runRecursive(*nodes_.at(edge), context, executor);
    }
    else
    {
      for (const auto& edge : node.getOutboundEdges())
        runRecursive(*nodes_.at(edge), context, executor);
    }
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

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerPipeline)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerPipeline)
