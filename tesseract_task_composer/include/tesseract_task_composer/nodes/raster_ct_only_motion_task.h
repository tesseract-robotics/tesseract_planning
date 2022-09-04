/**
 * @file raster_ct_only_motion_task.h
 * @brief Plans raster paths with cartesian transitions
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_CT_ONLY_MOTION_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_CT_ONLY_MOTION_TASK_H

#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_common/any_poly.h>

namespace tesseract_planning
{
/**
 * @brief The RasterCtOnlyMotionTask class
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
class RasterCtOnlyMotionTask : public TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<RasterCtOnlyMotionTask>;
  using ConstPtr = std::shared_ptr<const RasterCtOnlyMotionTask>;
  using UPtr = std::unique_ptr<RasterCtOnlyMotionTask>;
  using ConstUPtr = std::unique_ptr<const RasterCtOnlyMotionTask>;

  RasterCtOnlyMotionTask() = default;  // Required for serialization
  RasterCtOnlyMotionTask(std::string input_key,
                         std::string output_key,
                         bool is_conditional = true,
                         std::string name = "RasterCtOnlyMotionTask");
  ~RasterCtOnlyMotionTask() override = default;
  RasterCtOnlyMotionTask(const RasterCtOnlyMotionTask&) = delete;
  RasterCtOnlyMotionTask& operator=(const RasterCtOnlyMotionTask&) = delete;
  RasterCtOnlyMotionTask(RasterCtOnlyMotionTask&&) = delete;
  RasterCtOnlyMotionTask& operator=(RasterCtOnlyMotionTask&&) = delete;

  TaskComposerNode::UPtr clone() const override final;

  bool operator==(const RasterCtOnlyMotionTask& rhs) const;
  bool operator!=(const RasterCtOnlyMotionTask& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  TaskComposerNodeInfo::UPtr runImpl(TaskComposerInput& input,
                                     OptionalTaskComposerExecutor executor) const override final;

  static void checkTaskInput(const tesseract_common::AnyPoly& input);
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterCtOnlyMotionTask, "RasterCtOnlyMotionTask")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_CT_ONLY_MOTION_TASK_H
