/**
 * @file motion_planner_task.h
 * @brief Task Composer motion planner task
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
#ifndef TESSERACT_TASK_COMPOSER_MOTION_PLANNER_TASK_H
#define TESSERACT_TASK_COMPOSER_MOTION_PLANNER_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_info.h>
#include <tesseract_motion_planners/core/planner.h>

namespace tesseract_planning
{
class MotionPlannerTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<MotionPlannerTask>;
  using ConstPtr = std::shared_ptr<const MotionPlannerTask>;
  using UPtr = std::unique_ptr<MotionPlannerTask>;
  using ConstUPtr = std::unique_ptr<const MotionPlannerTask>;

  MotionPlannerTask() = default;  // Required for serialization
  MotionPlannerTask(MotionPlanner::Ptr planner,
                    std::string input_key,
                    std::string output_key,
                    bool format_result_as_input = true,
                    bool is_conditional = true);
  ~MotionPlannerTask() = default;

  int run(TaskComposerInput& input, OptionalTaskComposerExecutor executor = std::nullopt) const override;

  bool operator==(const MotionPlannerTask& rhs) const;
  bool operator!=(const MotionPlannerTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  MotionPlanner::Ptr planner_;
  std::string input_key_;
  std::string output_key_;
  bool format_result_as_input_;
};

class MotionPlannerTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<MotionPlannerTaskInfo>;
  using ConstPtr = std::shared_ptr<const MotionPlannerTaskInfo>;
  using UPtr = std::unique_ptr<MotionPlannerTaskInfo>;
  using ConstUPtr = std::unique_ptr<const MotionPlannerTaskInfo>;

  MotionPlannerTaskInfo() = default;
  MotionPlannerTaskInfo(boost::uuids::uuid uuid, std::string name);

  TaskComposerNodeInfo::UPtr clone() const override;

  bool operator==(const MotionPlannerTaskInfo& rhs) const;
  bool operator!=(const MotionPlannerTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::MotionPlannerTask, "MotionPlannerTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::MotionPlannerTaskInfo, "MotionPlannerTaskInfo")

#endif  // TESSERACT_TASK_COMPOSER_MOTION_PLANNER_TASK_H
