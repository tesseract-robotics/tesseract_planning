/**
 * @file descartes_motion_planner_task.h
 * @brief Descartes motion planning pipeline
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
#ifndef TESSERACT_TASK_COMPOSER_DESCARTES_MOTION_PLANNER_TASK_H
#define TESSERACT_TASK_COMPOSER_DESCARTES_MOTION_PLANNER_TASK_H

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_names.h>

#include <tesseract_task_composer/nodes/motion_planner_task.hpp>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>

namespace tesseract_planning
{
using DescartesMotionPlannerTaskBase = MotionPlannerTask<DescartesMotionPlannerF>;
class DescartesMotionPlannerTask : public DescartesMotionPlannerTaskBase
{
public:
  using Ptr = std::shared_ptr<DescartesMotionPlannerTask>;
  using ConstPtr = std::shared_ptr<const DescartesMotionPlannerTask>;
  using UPtr = std::unique_ptr<DescartesMotionPlannerTask>;
  using ConstUPtr = std::unique_ptr<const DescartesMotionPlannerTask>;

  DescartesMotionPlannerTask() = default;  // Required for serialization
  DescartesMotionPlannerTask(std::string input_key,
                             std::string output_key,
                             bool format_result_as_input = true,
                             bool is_conditional = true,
                             std::string name = node_names::DESCARTES_MOTION_PLANNER_TASK_NAME);
  ~DescartesMotionPlannerTask() override = default;
  DescartesMotionPlannerTask(const DescartesMotionPlannerTask&) = delete;
  DescartesMotionPlannerTask& operator=(const DescartesMotionPlannerTask&) = delete;
  DescartesMotionPlannerTask(DescartesMotionPlannerTask&&) = delete;
  DescartesMotionPlannerTask& operator=(DescartesMotionPlannerTask&&) = delete;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::DescartesMotionPlannerTaskBase, "DescartesMotionPlannerTaskBase")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::DescartesMotionPlannerTask, "DescartesMotionPlannerTask")

#endif  // TESSERACT_TASK_COMPOSER_DESCARTES_MOTION_PLANNER_TASK_H
