/**
 * @file upsample_trajectory_task_generator.h
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
#ifndef TESSERACT_PROCESS_MANAGERS_UPSAMPLE_TRAJECTORY_TASK_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_UPSAMPLE_TRAJECTORY_TASK_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_process_managers/core/default_task_namespaces.h>

namespace tesseract_planning
{
/**
 * @brief This is used to upsample the results trajectory based on the longest valid segment length.
 * @note This is primarily useful to run before running time parameterization, because motion planners
 * assume joint interpolated between states. If the points are spaced to fart apart the path between
 * two states may not be a straight line causing collision during execution.
 */
class UpsampleTrajectoryTaskGenerator : public TaskGenerator
{
public:
  using UPtr = std::unique_ptr<UpsampleTrajectoryTaskGenerator>;

  UpsampleTrajectoryTaskGenerator(std::string name = profile_ns::UPSAMPLE_TRAJECTORY_DEFAULT_NAMESPACE);

  ~UpsampleTrajectoryTaskGenerator() override = default;
  UpsampleTrajectoryTaskGenerator(const UpsampleTrajectoryTaskGenerator&) = delete;
  UpsampleTrajectoryTaskGenerator& operator=(const UpsampleTrajectoryTaskGenerator&) = delete;
  UpsampleTrajectoryTaskGenerator(UpsampleTrajectoryTaskGenerator&&) = delete;
  UpsampleTrajectoryTaskGenerator& operator=(UpsampleTrajectoryTaskGenerator&&) = delete;

  int conditionalProcess(TaskInput input, std::size_t unique_id) const override final;

  void process(TaskInput input, std::size_t unique_id) const override final;

private:
  void upsample(CompositeInstruction& composite,
                const CompositeInstruction& current_composite,
                InstructionPoly& start_instruction,
                double longest_valid_segment_length) const;
};

class UpsampleTrajectoryTaskInfo : public TaskInfo
{
public:
  using Ptr = std::shared_ptr<UpsampleTrajectoryTaskInfo>;
  using ConstPtr = std::shared_ptr<const UpsampleTrajectoryTaskInfo>;

  UpsampleTrajectoryTaskInfo() = default;
  UpsampleTrajectoryTaskInfo(std::size_t unique_id,
                             std::string name = profile_ns::UPSAMPLE_TRAJECTORY_DEFAULT_NAMESPACE);

  TaskInfo::UPtr clone() const override;

  bool operator==(const UpsampleTrajectoryTaskInfo& rhs) const;
  bool operator!=(const UpsampleTrajectoryTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::UpsampleTrajectoryTaskInfo, "UpsampleTrajectoryTaskInfo")
#endif  // TESSERACT_PROCESS_MANAGERS_UPSAMPLE_TRAJECTORY_TASK_GENERATOR_H
