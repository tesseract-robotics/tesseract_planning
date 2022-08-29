/**
 * @file raster_motion_task.h
 * @brief Raster motion pipeline
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_MOTION_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_MOTION_TASK_H

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_common/any.h>

namespace tesseract_planning
{
/**
 * @brief The RasterMotionTask class
 * @details The required format is below.
 *
 * Composite
 * {
 *   Composite - from start
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 *   Composite - Transitions
 *   Composite - Raster segment
 *   Composite - to end
 * }
 */
class RasterMotionTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<RasterMotionTask>;
  using ConstPtr = std::shared_ptr<const RasterMotionTask>;
  using UPtr = std::unique_ptr<RasterMotionTask>;
  using ConstUPtr = std::unique_ptr<const RasterMotionTask>;

  RasterMotionTask() = default;  // Required for serialization
  RasterMotionTask(std::string input_key,
                   std::string output_key,
                   bool is_conditional = true,
                   std::string name = "RasterMotionTask");
  ~RasterMotionTask() override = default;
  RasterMotionTask(const RasterMotionTask&) = delete;
  RasterMotionTask& operator=(const RasterMotionTask&) = delete;
  RasterMotionTask(RasterMotionTask&&) = delete;
  RasterMotionTask& operator=(RasterMotionTask&&) = delete;

  int run(TaskComposerInput& input, OptionalTaskComposerExecutor executor) const override final;

  TaskComposerNode::UPtr clone() const override final;

  bool operator==(const RasterMotionTask& rhs) const;
  bool operator!=(const RasterMotionTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  static void checkTaskInput(const tesseract_common::Any& input);
};

class RasterMotionTaskInfo : public TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<RasterMotionTaskInfo>;
  using ConstPtr = std::shared_ptr<const RasterMotionTaskInfo>;
  using UPtr = std::unique_ptr<RasterMotionTaskInfo>;
  using ConstUPtr = std::unique_ptr<const RasterMotionTaskInfo>;

  RasterMotionTaskInfo() = default;
  RasterMotionTaskInfo(boost::uuids::uuid uuid, std::string name);

  TaskComposerNodeInfo::UPtr clone() const override;

  bool operator==(const RasterMotionTaskInfo& rhs) const;
  bool operator!=(const RasterMotionTaskInfo& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterMotionTask, "RasterMotionTask")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterMotionTaskInfo, "RasterMotionTaskInfo")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_MOTION_TASK_H
