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

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_info.h>
#include <tesseract_task_composer/nodes/default_task_namespaces.h>

namespace tesseract_planning
{
/**
 * @brief This task modifies the const input instructions in order to push waypoints that are outside of their
 * limits back within them.
 */
class FixStateBoundsTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<FixStateBoundsTask>;
  using ConstPtr = std::shared_ptr<const FixStateBoundsTask>;
  using UPtr = std::unique_ptr<FixStateBoundsTask>;
  using ConstUPtr = std::unique_ptr<const FixStateBoundsTask>;

  FixStateBoundsTask() = default;  // Required for serialization
  FixStateBoundsTask(std::string input_key,
                     std::string output_key,
                     bool is_conditional = true,
                     std::string name = profile_ns::FIX_STATE_BOUNDS_DEFAULT_NAMESPACE);
  ~FixStateBoundsTask() override = default;
  FixStateBoundsTask(const FixStateBoundsTask&) = delete;
  FixStateBoundsTask& operator=(const FixStateBoundsTask&) = delete;
  FixStateBoundsTask(FixStateBoundsTask&&) = delete;
  FixStateBoundsTask& operator=(FixStateBoundsTask&&) = delete;

  int run(TaskComposerInput& input, OptionalTaskComposerExecutor executor = std::nullopt) const override final;

  TaskComposerNode::UPtr clone() const override final;

  bool operator==(const FixStateBoundsTask& rhs) const;
  bool operator!=(const FixStateBoundsTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  std::string input_key_;
  std::string output_key_;
};

class FixStateBoundsTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<FixStateBoundsTaskInfo>;
  using ConstPtr = std::shared_ptr<const FixStateBoundsTaskInfo>;
  using UPtr = std::unique_ptr<FixStateBoundsTaskInfo>;
  using ConstUPtr = std::unique_ptr<const FixStateBoundsTaskInfo>;

  FixStateBoundsTaskInfo() = default;
  FixStateBoundsTaskInfo(boost::uuids::uuid uuid, std::string name = profile_ns::FIX_STATE_BOUNDS_DEFAULT_NAMESPACE);

  TaskComposerNodeInfo::UPtr clone() const override;

  bool operator==(const FixStateBoundsTaskInfo& rhs) const;
  bool operator!=(const FixStateBoundsTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::FixStateBoundsTask, "FixStateBoundsTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::FixStateBoundsTaskInfo, "FixStateBoundsTaskInfo")
#endif  // TESSERACT_TASK_COMPOSER_FIX_STATE_BOUNDS_TASK_H
