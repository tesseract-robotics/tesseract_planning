/**
 * @file task_composer_pipeline.h
 * @brief A node in the pipeline
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PIPELINE_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PIPELINE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_graph.h>

namespace tesseract_planning
{
class TaskComposerContext;
class TaskComposerNode;
class TaskComposerNodeInfo;
class TaskComposerExecutor;
class TaskComposerPluginFactory;
/**
 * @brief This class facilitates the composition of an arbitrary taskflow pipeline.
 * Tasks are nodes in the graph connected to each other in a configurable order by directed edges
 */
class TaskComposerPipeline : public TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<TaskComposerPipeline>;
  using ConstPtr = std::shared_ptr<const TaskComposerPipeline>;
  using UPtr = std::unique_ptr<TaskComposerPipeline>;
  using ConstUPtr = std::unique_ptr<const TaskComposerPipeline>;

  TaskComposerPipeline(std::string name = "TaskComposerPipeline");
  TaskComposerPipeline(std::string name, bool conditional);
  TaskComposerPipeline(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& plugin_factory);
  ~TaskComposerPipeline() override = default;
  TaskComposerPipeline(const TaskComposerPipeline&) = delete;
  TaskComposerPipeline& operator=(const TaskComposerPipeline&) = delete;
  TaskComposerPipeline(TaskComposerPipeline&&) = delete;
  TaskComposerPipeline& operator=(TaskComposerPipeline&&) = delete;

  bool operator==(const TaskComposerPipeline& rhs) const;
  bool operator!=(const TaskComposerPipeline& rhs) const;

private:
  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override final;

  void runRecursive(const TaskComposerNode& node,
                    TaskComposerContext& context,
                    OptionalTaskComposerExecutor executor = std::nullopt) const;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TaskComposerPipeline)

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PIPELINE_H
