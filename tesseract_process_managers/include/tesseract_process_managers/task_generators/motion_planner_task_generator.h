/**
 * @file motion_planner_task_generator.h
 * @brief Generates a motion planning process
 *
 * @author Matthew Powelson
 * @date July 15. 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_MOTION_PLANNER_TASK_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_MOTION_PLANNER_TASK_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_generator.h>

namespace tesseract_planning
{
// Forward Declare
class MotionPlanner;

class MotionPlannerTaskGenerator : public TaskGenerator
{
public:
  using UPtr = std::unique_ptr<MotionPlannerTaskGenerator>;

  MotionPlannerTaskGenerator(std::shared_ptr<MotionPlanner> planner);
  ~MotionPlannerTaskGenerator() override = default;
  MotionPlannerTaskGenerator(const MotionPlannerTaskGenerator&) = delete;
  MotionPlannerTaskGenerator& operator=(const MotionPlannerTaskGenerator&) = delete;
  MotionPlannerTaskGenerator(MotionPlannerTaskGenerator&&) = delete;
  MotionPlannerTaskGenerator& operator=(MotionPlannerTaskGenerator&&) = delete;

  int conditionalProcess(TaskInput input, std::size_t unique_id) const override final;

  void process(TaskInput input, std::size_t unique_id) const override final;

private:
  std::shared_ptr<MotionPlanner> planner_{ nullptr };
};

class MotionPlannerTaskInfo : public TaskInfo
{
public:
  using Ptr = std::shared_ptr<MotionPlannerTaskInfo>;
  using ConstPtr = std::shared_ptr<const MotionPlannerTaskInfo>;

  MotionPlannerTaskInfo() = default;
  MotionPlannerTaskInfo(std::size_t unique_id, std::string name);

  TaskInfo::UPtr clone() const override;

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
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::MotionPlannerTaskInfo, "MotionPlannerTaskInfo")
#endif
