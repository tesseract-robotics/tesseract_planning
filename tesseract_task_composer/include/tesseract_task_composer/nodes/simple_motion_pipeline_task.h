/**
 * @file simple_motion_planner_task.h
 * @brief simple motion planning pipeline
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
#ifndef TESSERACT_TASK_COMPOSER_SIMPLE_MOTION_PIPELINE_TASK_H
#define TESSERACT_TASK_COMPOSER_SIMPLE_MOTION_PIPELINE_TASK_H

#include <tesseract_task_composer/task_composer_graph.h>

namespace tesseract_planning
{
class SimpleMotionPipelineTask : public TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<SimpleMotionPipelineTask>;
  using ConstPtr = std::shared_ptr<const SimpleMotionPipelineTask>;
  using UPtr = std::unique_ptr<SimpleMotionPipelineTask>;
  using ConstUPtr = std::unique_ptr<const SimpleMotionPipelineTask>;

  /**
   * @brief SimpleMotionPipelineTask
   * @details This will use the uuid as the input and output key
   * @param name The name give to the task
   */
  SimpleMotionPipelineTask(std::string name = "SimpleMotionPipelineTask");
  SimpleMotionPipelineTask(std::string input_key,
                           std::string output_key,
                           std::string name = "SimpleMotionPipelineTask");
  ~SimpleMotionPipelineTask() override = default;
  SimpleMotionPipelineTask(const SimpleMotionPipelineTask&) = delete;
  SimpleMotionPipelineTask& operator=(const SimpleMotionPipelineTask&) = delete;
  SimpleMotionPipelineTask(SimpleMotionPipelineTask&&) = delete;
  SimpleMotionPipelineTask& operator=(SimpleMotionPipelineTask&&) = delete;

  bool operator==(const SimpleMotionPipelineTask& rhs) const;
  bool operator!=(const SimpleMotionPipelineTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  void ctor(std::string input_key, std::string output_key);
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::SimpleMotionPipelineTask, "SimpleMotionPipelineTask")

#endif  // TESSERACT_TASK_COMPOSER_SIMPLE_MOTION_PIPELINE_TASK_H
