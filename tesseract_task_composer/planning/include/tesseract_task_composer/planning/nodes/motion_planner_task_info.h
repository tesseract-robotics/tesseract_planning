/**
 * @file motion_planner_task_info.h
 * @brief Task Composer motion planner task info
 *
 * @author Levi Armstrong
 * @date June 8. 2023
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
#ifndef TESSERACT_TASK_COMPOSER_MOTION_PLANNER_TASK_INFO_H
#define TESSERACT_TASK_COMPOSER_MOTION_PLANNER_TASK_INFO_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

#include <tesseract_environment/environment.h>

namespace tesseract_planning
{
class MotionPlannerTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<MotionPlannerTaskInfo>;
  using ConstPtr = std::shared_ptr<const MotionPlannerTaskInfo>;
  using UPtr = std::unique_ptr<MotionPlannerTaskInfo>;
  using ConstUPtr = std::unique_ptr<const MotionPlannerTaskInfo>;

  MotionPlannerTaskInfo() = default;
  MotionPlannerTaskInfo(const TaskComposerTask& task);

  tesseract_environment::Environment::ConstPtr env;

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
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::MotionPlannerTaskInfo, "MotionPlannerTaskInfo")
#endif  // TESSERACT_TASK_COMPOSER_MOTION_PLANNER_TASK_INFO_H
