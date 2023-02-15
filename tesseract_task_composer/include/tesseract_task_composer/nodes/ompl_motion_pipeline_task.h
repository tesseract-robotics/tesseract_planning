/**
 * @file ompl_motion_planner_task.h
 * @brief OMPL motion planning pipeline
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
#ifndef TESSERACT_TASK_COMPOSER_OMPL_MOTION_PIPELINE_TASK_H
#define TESSERACT_TASK_COMPOSER_OMPL_MOTION_PIPELINE_TASK_H

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_node_names.h>

#include <tesseract_task_composer/nodes/motion_pipeline_task.hpp>
#include <tesseract_task_composer/nodes/ompl_motion_planner_task.h>
#include <tesseract_task_composer/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/nodes/iterative_spline_parameterization_task.h>

namespace tesseract_planning
{
using OMPLMotionPipelineTaskBase =
    MotionPipelineTask<OMPLMotionPlannerTask, DiscreteContactCheckTask, IterativeSplineParameterizationTask>;
class OMPLMotionPipelineTask : public OMPLMotionPipelineTaskBase
{
public:
  using Ptr = std::shared_ptr<OMPLMotionPipelineTask>;
  using ConstPtr = std::shared_ptr<const OMPLMotionPipelineTask>;
  using UPtr = std::unique_ptr<OMPLMotionPipelineTask>;
  using ConstUPtr = std::unique_ptr<const OMPLMotionPipelineTask>;

  OMPLMotionPipelineTask(std::string name = node_names::OMPL_PIPELINE_NAME);
  OMPLMotionPipelineTask(std::string input_key,
                         std::string output_key,
                         std::string name = node_names::OMPL_PIPELINE_NAME);
  ~OMPLMotionPipelineTask() override = default;
  OMPLMotionPipelineTask(const OMPLMotionPipelineTask&) = delete;
  OMPLMotionPipelineTask& operator=(const OMPLMotionPipelineTask&) = delete;
  OMPLMotionPipelineTask(OMPLMotionPipelineTask&&) = delete;
  OMPLMotionPipelineTask& operator=(OMPLMotionPipelineTask&&) = delete;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::OMPLMotionPipelineTaskBase, "OMPLMotionPipelineTaskBase")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::OMPLMotionPipelineTask, "OMPLMotionPipelineTask")

#endif  // TESSERACT_TASK_COMPOSER_OMPL_MOTION_PIPELINE_TASK_H
