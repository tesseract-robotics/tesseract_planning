/**
 * @file ruckig_trajectory_smoothing_task_generator.h
 * @brief Leveraging Ruckig to smooth trajectory
 *
 * @author Levi Armstrong
 * @date July 27, 2022
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_PROCESS_MANAGERS_RUCKIG_TRAJECTORY_SMOOTHING_TASK_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_RUCKIG_TRAJECTORY_SMOOTHING_TASK_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_generator.h>
#include <tesseract_time_parameterization/ruckig_trajectory_smoothing.h>
#include <tesseract_process_managers/core/default_task_namespaces.h>

namespace tesseract_planning
{
class RuckigTrajectorySmoothingTaskGenerator : public TaskGenerator
{
public:
  using UPtr = std::unique_ptr<RuckigTrajectorySmoothingTaskGenerator>;

  RuckigTrajectorySmoothingTaskGenerator(std::string name = profile_ns::RUCKIG_TRAJECTORY_SMOOTHING_DEFAULT_NAMESPACE);

  ~RuckigTrajectorySmoothingTaskGenerator() override = default;
  RuckigTrajectorySmoothingTaskGenerator(const RuckigTrajectorySmoothingTaskGenerator&) = delete;
  RuckigTrajectorySmoothingTaskGenerator& operator=(const RuckigTrajectorySmoothingTaskGenerator&) = delete;
  RuckigTrajectorySmoothingTaskGenerator(RuckigTrajectorySmoothingTaskGenerator&&) = delete;
  RuckigTrajectorySmoothingTaskGenerator& operator=(RuckigTrajectorySmoothingTaskGenerator&&) = delete;

  int conditionalProcess(TaskInput input, std::size_t unique_id) const override final;

  void process(TaskInput input, std::size_t unique_id) const override final;
};

class RuckigTrajectorySmoothingTaskInfo : public TaskInfo
{
public:
  using Ptr = std::shared_ptr<RuckigTrajectorySmoothingTaskInfo>;
  using ConstPtr = std::shared_ptr<const RuckigTrajectorySmoothingTaskInfo>;

  RuckigTrajectorySmoothingTaskInfo() = default;
  RuckigTrajectorySmoothingTaskInfo(std::size_t unique_id,
                                    std::string name = profile_ns::RUCKIG_TRAJECTORY_SMOOTHING_DEFAULT_NAMESPACE);

  TaskInfo::UPtr clone() const override;

  bool operator==(const RuckigTrajectorySmoothingTaskInfo& rhs) const;
  bool operator!=(const RuckigTrajectorySmoothingTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RuckigTrajectorySmoothingTaskInfo, "RuckigTrajectorySmoothingTaskInfo")

#endif  // TESSERACT_PROCESS_MANAGERS_RUCKIG_TRAJECTORY_SMOOTHING_TASK_GENERATOR_H
