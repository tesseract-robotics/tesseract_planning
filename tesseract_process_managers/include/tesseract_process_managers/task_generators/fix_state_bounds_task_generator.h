/**
 * @file fix_state_bounds_task_generator.h
 * @brief Process generator for process that pushes plan instructions back within joint limits
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
#ifndef TESSERACT_PROCESS_MANAGERS_FIX_STATE_BOUNDS_TASK_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_FIX_STATE_BOUNDS_TASK_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_process_managers/core/default_task_namespaces.h>

namespace tesseract_planning
{
/**
 * @brief This generator modifies the const input instructions in order to push waypoints that are outside of their
 * limits back within them.
 */
class FixStateBoundsTaskGenerator : public TaskGenerator
{
public:
  using UPtr = std::unique_ptr<FixStateBoundsTaskGenerator>;

  FixStateBoundsTaskGenerator(std::string name = profile_ns::FIX_STATE_BOUNDS_DEFAULT_NAMESPACE);

  ~FixStateBoundsTaskGenerator() override = default;
  FixStateBoundsTaskGenerator(const FixStateBoundsTaskGenerator&) = delete;
  FixStateBoundsTaskGenerator& operator=(const FixStateBoundsTaskGenerator&) = delete;
  FixStateBoundsTaskGenerator(FixStateBoundsTaskGenerator&&) = delete;
  FixStateBoundsTaskGenerator& operator=(FixStateBoundsTaskGenerator&&) = delete;

  int conditionalProcess(TaskInput input, std::size_t unique_id) const override final;

  void process(TaskInput input, std::size_t unique_id) const override final;
};

class FixStateBoundsTaskInfo : public TaskInfo
{
public:
  using Ptr = std::shared_ptr<FixStateBoundsTaskInfo>;
  using ConstPtr = std::shared_ptr<const FixStateBoundsTaskInfo>;

  FixStateBoundsTaskInfo() = default;
  FixStateBoundsTaskInfo(std::size_t unique_id, std::string name = profile_ns::FIX_STATE_BOUNDS_DEFAULT_NAMESPACE);

  TaskInfo::UPtr clone() const override;

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
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::FixStateBoundsTaskInfo, "FixStateBoundsTaskInfo")
#endif  // TESSERACT_PROCESS_MANAGERS_FIX_STATE_BOUNDS_TASK_GENERATOR_H
