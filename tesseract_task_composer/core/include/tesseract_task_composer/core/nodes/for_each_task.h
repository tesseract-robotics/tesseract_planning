/**
 * @file for_each_task.h
 * @brief This runs the same task for each element in a vector
 *
 * @author Matthew Powelson
 * @date July 15, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_TASK_COMPOSER_FOR_EACH_TASK_H
#define TESSERACT_TASK_COMPOSER_FOR_EACH_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_common/fwd.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;

class ForEachTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INOUT_PORT;

  struct TaskFactoryResults
  {
    TaskComposerNode::UPtr node;
    std::string input_key;
    std::string output_key;
  };
  using TaskFactory = std::function<TaskFactoryResults(const std::string& name, std::size_t index)>;

  ForEachTask();
  explicit ForEachTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& plugin_factory);

  ~ForEachTask() override = default;
  ForEachTask(const ForEachTask&) = delete;
  ForEachTask& operator=(const ForEachTask&) = delete;
  ForEachTask(ForEachTask&&) = delete;
  ForEachTask& operator=(ForEachTask&&) = delete;

  bool operator==(const ForEachTask& rhs) const;
  bool operator!=(const ForEachTask& rhs) const;

private:
  TaskFactory task_factory_;
  std::string task_input_port_;
  std::string task_output_port_;

  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor) const override final;

  static void checkTaskInput(const tesseract_common::AnyPoly& input);

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::ForEachTask)

#endif  // TESSERACT_TASK_COMPOSER_FOR_EACH_TASK_H
