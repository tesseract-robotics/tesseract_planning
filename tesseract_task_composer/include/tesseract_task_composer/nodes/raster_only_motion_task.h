/**
 * @file raster_only_motion_task.h
 * @brief Plans raster paths
 *
 * @author Matthew Powelson
 * @date July 15, 2020
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_ONLY_MOTION_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_ONLY_MOTION_TASK_H

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_common/any.h>

namespace tesseract_planning
{
/**
 * @brief The RasterOnlyMotionTask class
 * @details The required format is below.
 *
 * Composite
 * {
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 * }
 */
class RasterOnlyMotionTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<RasterOnlyMotionTask>;
  using ConstPtr = std::shared_ptr<const RasterOnlyMotionTask>;
  using UPtr = std::unique_ptr<RasterOnlyMotionTask>;
  using ConstUPtr = std::unique_ptr<const RasterOnlyMotionTask>;

  RasterOnlyMotionTask() = default;  // Required for serialization
  RasterOnlyMotionTask(std::string input_key,
                       std::string output_key,
                       bool cartesian_transition = false,
                       bool run_simple_planner = true,
                       bool is_conditional = true,
                       std::string name = "RasterOnlyMotionTask");
  ~RasterOnlyMotionTask() override = default;
  RasterOnlyMotionTask(const RasterOnlyMotionTask&) = delete;
  RasterOnlyMotionTask& operator=(const RasterOnlyMotionTask&) = delete;
  RasterOnlyMotionTask(RasterOnlyMotionTask&&) = delete;
  RasterOnlyMotionTask& operator=(RasterOnlyMotionTask&&) = delete;

  int run(TaskComposerInput& input, OptionalTaskComposerExecutor executor) const override final;

  TaskComposerNode::UPtr clone() const override final;

  bool operator==(const RasterOnlyMotionTask& rhs) const;
  bool operator!=(const RasterOnlyMotionTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  bool run_simple_planner_{ true };
  bool cartesian_transition_{ false };

  static void checkTaskInput(const tesseract_common::Any& input);
};

class RasterOnlyMotionTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<RasterOnlyMotionTaskInfo>;
  using ConstPtr = std::shared_ptr<const RasterOnlyMotionTaskInfo>;
  using UPtr = std::unique_ptr<RasterOnlyMotionTaskInfo>;
  using ConstUPtr = std::unique_ptr<const RasterOnlyMotionTaskInfo>;

  RasterOnlyMotionTaskInfo() = default;
  RasterOnlyMotionTaskInfo(boost::uuids::uuid uuid, std::string name);

  TaskComposerNodeInfo::UPtr clone() const override;

  bool operator==(const RasterOnlyMotionTaskInfo& rhs) const;
  bool operator!=(const RasterOnlyMotionTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterOnlyMotionTask, "RasterOnlyMotionTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterOnlyMotionTaskInfo, "RasterOnlyMotionTaskInfo")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_ONLY_MOTION_TASK_H
