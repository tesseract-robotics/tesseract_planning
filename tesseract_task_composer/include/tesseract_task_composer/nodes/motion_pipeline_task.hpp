/**
 * @file motion_pipeline_task.h
 * @brief Task Composer motion pipeline task
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
#ifndef TESSERACT_TASK_COMPOSER_MOTION_PIPELINE_TASK_HPP
#define TESSERACT_TASK_COMPOSER_MOTION_PIPELINE_TASK_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/nodes/min_length_task.h>
#include <tesseract_task_composer/nodes/done_task.h>
#include <tesseract_task_composer/nodes/error_task.h>

namespace tesseract_planning
{
template <typename MotionPlannerTaskType, typename ContactCheckTaskType, typename TimeParameterizationTaskType>
class MotionPipelineTask : public TaskComposerGraph
{
public:
  MotionPipelineTask() = default;  // Required for serialization
  /**
   * @brief DescartesMotionPipelineTask
   * @details This will use the uuid as the input and output key
   * @param name The name give to the task
   */
  // NOLINTNEXTLINE(performance-unnecessary-value-param)
  MotionPipelineTask(std::string name) : TaskComposerGraph(std::move(name)) { ctor(uuid_str_, uuid_str_); }
  // NOLINTNEXTLINE(performance-unnecessary-value-param)
  MotionPipelineTask(std::string input_key, std::string output_key, std::string name)
    : TaskComposerGraph(std::move(name))
  {
    ctor(std::move(input_key), std::move(output_key));
  }
  ~MotionPipelineTask() override = default;
  MotionPipelineTask(const MotionPipelineTask&) = delete;
  MotionPipelineTask& operator=(const MotionPipelineTask&) = delete;
  MotionPipelineTask(MotionPipelineTask&&) = delete;
  MotionPipelineTask& operator=(MotionPipelineTask&&) = delete;

  bool operator==(const MotionPipelineTask& rhs) const
  {
    bool equal = true;
    equal &= TaskComposerGraph::operator==(rhs);
    return equal;
  }
  bool operator!=(const MotionPipelineTask& rhs) const { return !operator==(rhs); }

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
  }

  void ctor(std::string input_key, std::string output_key)
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));

    boost::uuids::uuid done_task = addNode(std::make_unique<DoneTask>());
    boost::uuids::uuid error_task = addNode(std::make_unique<ErrorTask>());

    // Setup Min Length Process Generator
    // This is required because trajopt requires a minimum length trajectory.
    // This is used to correct the seed if it is to short.
    boost::uuids::uuid min_length_task = addNode(std::make_unique<MinLengthTask>(input_keys_[0], output_keys_[0]));

    // Setup TrajOpt
    boost::uuids::uuid motion_planner_task =
        addNode(std::make_unique<MotionPlannerTaskType>(output_keys_[0], output_keys_[0], false));

    // Setup post collision check
    boost::uuids::uuid contact_check_task = addNode(std::make_unique<ContactCheckTaskType>(output_keys_[0]));

    // Setup time parameterization and smoothing
    boost::uuids::uuid time_parameterization_task =
        addNode(std::make_unique<TimeParameterizationTaskType>(output_keys_[0], output_keys_[0]));

    // Add edges
    addEdges(min_length_task, { motion_planner_task });
    addEdges(motion_planner_task, { error_task, contact_check_task });
    addEdges(contact_check_task, { error_task, time_parameterization_task });
    addEdges(time_parameterization_task, { error_task, done_task });
  }
};
}  // namespace tesseract_planning
#endif  // TESSERACT_TASK_COMPOSER_MOTION_PIPELINE_TASK_HPP
