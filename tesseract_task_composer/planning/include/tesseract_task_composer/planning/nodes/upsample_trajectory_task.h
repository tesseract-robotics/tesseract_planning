/**
 * @file upsample_trajectory_task.h
 *
 * @author Levi Armstrong
 * @date December 15, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSER_UPSAMPLE_TRAJECTORY_TASK_H
#define TESSERACT_TASK_COMPOSER_UPSAMPLE_TRAJECTORY_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
/**
 * @brief This is used to upsample the results trajectory based on the longest valid segment length.
 * @note This is primarily useful to run before running time parameterization, because motion planners
 * assume joint interpolated between states. If the points are spaced to fart apart the path between
 * two states may not be a straight line causing collision during execution.
 */
class UpsampleTrajectoryTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<UpsampleTrajectoryTask>;
  using ConstPtr = std::shared_ptr<const UpsampleTrajectoryTask>;
  using UPtr = std::unique_ptr<UpsampleTrajectoryTask>;
  using ConstUPtr = std::unique_ptr<const UpsampleTrajectoryTask>;

  UpsampleTrajectoryTask();
  explicit UpsampleTrajectoryTask(std::string name,
                                  std::string input_key,
                                  std::string output_key,
                                  bool conditional = false);
  explicit UpsampleTrajectoryTask(std::string name,
                                  const YAML::Node& config,
                                  const TaskComposerPluginFactory& plugin_factory);
  ~UpsampleTrajectoryTask() override = default;
  UpsampleTrajectoryTask(const UpsampleTrajectoryTask&) = delete;
  UpsampleTrajectoryTask& operator=(const UpsampleTrajectoryTask&) = delete;
  UpsampleTrajectoryTask(UpsampleTrajectoryTask&&) = delete;
  UpsampleTrajectoryTask& operator=(UpsampleTrajectoryTask&&) = delete;

  bool operator==(const UpsampleTrajectoryTask& rhs) const;
  bool operator!=(const UpsampleTrajectoryTask& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  void upsample(CompositeInstruction& composite,
                const CompositeInstruction& current_composite,
                InstructionPoly& start_instruction,
                double longest_valid_segment_length) const;

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerContext& context,
                                     OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::UpsampleTrajectoryTask, "UpsampleTrajectoryTask")
#endif  // TESSERACT_TASK_COMPOSER_UPSAMPLE_TRAJECTORY_TASK_H
