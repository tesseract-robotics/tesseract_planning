/**
 * @file format_as_input_task.h
 *
 * @brief This is used in the case where you run trajopt with collision as a cost and then you post check it for
 * collision and it fails. Then you run trajopt with collision as a constraint but the output from trajopt with
 * collision as a cost must be formated as input for trajopt with collision as a constraint planner.
 *
 * @author Levi Armstrong
 * @date April 6. 2023
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
#ifndef TESSERACT_TASK_COMPOSER_FORMAT_AS_INPUT_TASK_H
#define TESSERACT_TASK_COMPOSER_FORMAT_AS_INPUT_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
/**
 * @brief This is used in the case where you run trajopt with collision as a cost and then you post check it for
 * collision and it fails. Then you run trajopt with collision as a constraint but the output from trajopt with
 * collision as a cost must be formated as input for trajopt with collision as a constraint planner.
 *
 * This will take the results stored in input_unformated_key and store it in the input_formatted_key program and
 * save the results in the output key.
 *
 * input_keys[0]: The original input to motion planning
 * input_keys[1]: The output of the first motion plan which failed collision checking
 */
class FormatAsInputTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<FormatAsInputTask>;
  using ConstPtr = std::shared_ptr<const FormatAsInputTask>;
  using UPtr = std::unique_ptr<FormatAsInputTask>;
  using ConstUPtr = std::unique_ptr<const FormatAsInputTask>;

  FormatAsInputTask();
  explicit FormatAsInputTask(std::string name,
                             const std::array<std::string, 2>& input_keys,
                             std::string output_key,
                             bool conditional = true);
  explicit FormatAsInputTask(std::string name,
                             const YAML::Node& config,
                             const TaskComposerPluginFactory& plugin_factory);
  ~FormatAsInputTask() override = default;
  FormatAsInputTask(const FormatAsInputTask&) = delete;
  FormatAsInputTask& operator=(const FormatAsInputTask&) = delete;
  FormatAsInputTask(FormatAsInputTask&&) = delete;
  FormatAsInputTask& operator=(FormatAsInputTask&&) = delete;

  bool operator==(const FormatAsInputTask& rhs) const;
  bool operator!=(const FormatAsInputTask& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerContext& context,
                                     OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::FormatAsInputTask, "FormatAsInputTask")
#endif  // TESSERACT_TASK_COMPOSER_FORMAT_AS_INPUT_TASK_H
