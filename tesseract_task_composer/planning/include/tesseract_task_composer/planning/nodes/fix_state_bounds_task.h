/**
 * @file fix_state_bounds_task.h
 * @brief Task that pushes plan instructions back within joint limits
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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
#ifndef TESSERACT_TASK_COMPOSER_FIX_STATE_BOUNDS_TASK_H
#define TESSERACT_TASK_COMPOSER_FIX_STATE_BOUNDS_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;

/**
 * @brief This task modifies the input instructions in order to push waypoints that are outside of their
 * limits back within them.
 */
class FixStateBoundsTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<FixStateBoundsTask>;
  using ConstPtr = std::shared_ptr<const FixStateBoundsTask>;
  using UPtr = std::unique_ptr<FixStateBoundsTask>;
  using ConstUPtr = std::unique_ptr<const FixStateBoundsTask>;

  FixStateBoundsTask();
  explicit FixStateBoundsTask(std::string name, std::string input_key, std::string output_key, bool conditional = true);
  explicit FixStateBoundsTask(std::string name,
                              const YAML::Node& config,
                              const TaskComposerPluginFactory& plugin_factory);
  ~FixStateBoundsTask() override = default;
  FixStateBoundsTask(const FixStateBoundsTask&) = delete;
  FixStateBoundsTask& operator=(const FixStateBoundsTask&) = delete;
  FixStateBoundsTask(FixStateBoundsTask&&) = delete;
  FixStateBoundsTask& operator=(FixStateBoundsTask&&) = delete;

  bool operator==(const FixStateBoundsTask& rhs) const;
  bool operator!=(const FixStateBoundsTask& rhs) const;

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
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::FixStateBoundsTask, "FixStateBoundsTask")
#endif  // TESSERACT_TASK_COMPOSER_FIX_STATE_BOUNDS_TASK_H
