/**
 * @file trajopt_ifopt_motion_planner_task.h
 * @brief TrajOpt Ifopt motion planning pipeline
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
#ifndef TESSERACT_TASK_COMPOSER_TRAJOPT_IFOPT_MOTION_PLANNER_TASK_H
#define TESSERACT_TASK_COMPOSER_TRAJOPT_IFOPT_MOTION_PLANNER_TASK_H

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_node_names.h>

#include <tesseract_task_composer/nodes/motion_planner_task.hpp>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h>

namespace tesseract_planning
{
using TrajOptIfoptMotionPlannerTaskBase = MotionPlannerTask<TrajOptIfoptMotionPlanner>;
class TrajOptIfoptMotionPlannerTask : public TrajOptIfoptMotionPlannerTaskBase
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptMotionPlannerTask>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptMotionPlannerTask>;
  using UPtr = std::unique_ptr<TrajOptIfoptMotionPlannerTask>;
  using ConstUPtr = std::unique_ptr<const TrajOptIfoptMotionPlannerTask>;

  TrajOptIfoptMotionPlannerTask() = default;  // Required for serialization
  TrajOptIfoptMotionPlannerTask(std::string input_key,
                                std::string output_key,
                                bool format_result_as_input = true,
                                bool is_conditional = true,
                                std::string name = node_names::TRAJOPT_IFOPT_MOTION_PLANNER_TASK_NAME);
  ~TrajOptIfoptMotionPlannerTask() override = default;
  TrajOptIfoptMotionPlannerTask(const TrajOptIfoptMotionPlannerTask&) = delete;
  TrajOptIfoptMotionPlannerTask& operator=(const TrajOptIfoptMotionPlannerTask&) = delete;
  TrajOptIfoptMotionPlannerTask(TrajOptIfoptMotionPlannerTask&&) = delete;
  TrajOptIfoptMotionPlannerTask& operator=(TrajOptIfoptMotionPlannerTask&&) = delete;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TrajOptIfoptMotionPlannerTaskBase, "TrajOptIfoptMotionPlannerTaskBase")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TrajOptIfoptMotionPlannerTask, "TrajOptIfoptMotionPlannerTask")

#endif  // TESSERACT_TASK_COMPOSER_TRAJOPT_IFOPT_MOTION_PLANNER_TASK_H
