/**
 * @file ruckig_trajectory_smoothing_task.h
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
#ifndef TESSERACT_TASK_COMPOSER_RUCKIG_TRAJECTORY_SMOOTHING_TASK_H
#define TESSERACT_TASK_COMPOSER_RUCKIG_TRAJECTORY_SMOOTHING_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_names.h>

namespace tesseract_planning
{
class RuckigTrajectorySmoothingTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<RuckigTrajectorySmoothingTask>;
  using ConstPtr = std::shared_ptr<const RuckigTrajectorySmoothingTask>;
  using UPtr = std::unique_ptr<RuckigTrajectorySmoothingTask>;
  using ConstUPtr = std::unique_ptr<const RuckigTrajectorySmoothingTask>;

  RuckigTrajectorySmoothingTask() = default;  // Required for serialization
  RuckigTrajectorySmoothingTask(std::string input_key,
                                std::string output_key,
                                bool is_conditional = true,
                                std::string name = node_names::RUCKIG_TRAJECTORY_SMOOTHING_TASK_NAME);
  ~RuckigTrajectorySmoothingTask() override = default;
  RuckigTrajectorySmoothingTask(const RuckigTrajectorySmoothingTask&) = delete;
  RuckigTrajectorySmoothingTask& operator=(const RuckigTrajectorySmoothingTask&) = delete;
  RuckigTrajectorySmoothingTask(RuckigTrajectorySmoothingTask&&) = delete;
  RuckigTrajectorySmoothingTask& operator=(RuckigTrajectorySmoothingTask&&) = delete;

  bool operator==(const RuckigTrajectorySmoothingTask& rhs) const;
  bool operator!=(const RuckigTrajectorySmoothingTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerInput& input,
                                     OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RuckigTrajectorySmoothingTask, "RuckigTrajectorySmoothingTask")

#endif  // TESSERACT_TASK_COMPOSER_RUCKIG_TRAJECTORY_SMOOTHING_TASK_H
